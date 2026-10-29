#include <stdint.h>

#include <generated/csr.h>

#define STATUS_LINK_READY (1u << 0)
#define STATUS_DONE       (1u << 2)
#define STATUS_ERROR      (1u << 3)

#define LINE_MAX          128

void isr(void);
int main(void);

static char line_buffer[LINE_MAX];

void isr(void)
{
}

static void uart_putc(char c)
{
	while (uart_txfull_read())
		;
	uart_rxtx_write((uint8_t)c);
}

static char uart_getc(void)
{
	while (uart_rxempty_read())
		;
	return (char)uart_rxtx_read();
}

static void uart_puts(const char *s)
{
	while (*s)
		uart_putc(*s++);
}

static void put_hex32(uint32_t value, int digits)
{
	int shift = (digits - 1) * 4;

	while (shift >= 0) {
		int nibble = (value >> shift) & 0xfu;

		uart_putc(nibble < 10 ? '0' + nibble : 'a' + nibble - 10);
		shift -= 4;
	}
}

static void put_hex64(uint64_t value)
{
	put_hex32((uint32_t)(value >> 32), 8);
	put_hex32((uint32_t)value, 8);
}

static void print_status(void)
{
	uart_puts("chunk=0x");
	put_hex32(chunk_dma_status_read(), 2);
	uart_puts(" link=0x");
	put_hex32(pcie_phy_link_status_read(), 8);
	uart_puts(" bme=");
	uart_putc((pcie_phy_bus_master_enable_read() & 1) ? '1' : '0');
	uart_puts(" id=0x");
	put_hex32(main_pcie_id_read(), 4);
	uart_puts(" dbg=0x");
	put_hex64(chunk_dma_debug_read());
	uart_puts("\r\n");
}

static int str_eq(const char *a, const char *b)
{
	while (*a && *b) {
		if (*a != *b)
			return 0;
		a++;
		b++;
	}
	return *a == *b;
}

static int hex_digit(char c)
{
	if (c >= '0' && c <= '9')
		return c - '0';
	if (c >= 'a' && c <= 'f')
		return c - 'a' + 10;
	if (c >= 'A' && c <= 'F')
		return c - 'A' + 10;
	return -1;
}

static int parse_hex64(const char *text, uint32_t *high, uint32_t *low)
{
	uint32_t hi = 0;
	uint32_t lo = 0;
	int digits = 0;

	if (text[0] == '0' && (text[1] == 'x' || text[1] == 'X'))
		text += 2;
	if (*text == '\0')
		return -1;

	while (*text) {
		int digit = hex_digit(*text++);

		if (digit < 0)
			return -1;
		if (digits == 16)
			return -1;
		if (hi > 0x0fffffffu)
			return -1;
		hi = (hi << 4) | (lo >> 28);
		lo = (lo << 4) | (uint32_t)digit;
		digits++;
	}

	*high = hi;
	*low = lo;
	return 0;
}

static int tokenize(char *text, char **argv, int max_args)
{
	int argc = 0;

	for (;;) {
		while (*text == ' ' || *text == '\t')
			text++;
		if (*text == '\0')
			break;
		if (argc == max_args)
			return -1;
		argv[argc++] = text;
		while (*text && *text != ' ' && *text != '\t')
			text++;
		if (*text)
			*text++ = '\0';
	}
	return argc;
}

static int readline(char *buffer, int size)
{
	int count = 0;
	int overflow = 0;

	for (;;) {
		char c = uart_getc();

		if (c == '\r' || c == '\n') {
			uart_puts("\r\n");
			if (overflow)
				return -1;
			buffer[count] = '\0';
			return count;
		}
		if (c == '\b' || c == 0x7f) {
			if (!overflow && count > 0) {
				count--;
				uart_puts("\b \b");
			}
			continue;
		}
		if ((unsigned char)c < 0x20 || (unsigned char)c > 0x7e)
			continue;
		if (overflow)
			continue;
		if (count >= size - 1) {
			overflow = 1;
			continue;
		}
		buffer[count++] = c;
		uart_putc(c);
	}
}

static void print_help(void)
{
	uart_puts("commands:\r\n");
	uart_puts("  help                show this help\r\n");
	uart_puts("  status              show PCIe/DMA status\r\n");
	uart_puts("  scan <addr64> <len64> <w0> <w1> <w2> <w3>\r\n");
	uart_puts("  peek                show first 16 bytes of DMA buffer 0\r\n");
	uart_puts("                      find a 16-byte-aligned 128-bit pattern\r\n");
	uart_puts("WARNING: arbitrary physical reads may hang or crash the host!\r\n");
}

static int wait_chunk(void)
{
	uint32_t status;

	do {
		status = chunk_dma_status_read();
	} while (!(status & (STATUS_DONE | STATUS_ERROR)));

	if (status & STATUS_ERROR) {
		uart_puts("error: chunk dma 0x");
		put_hex32(chunk_dma_error_read(), 8);
		uart_puts("\r\n");
		return -1;
	}
	return 0;
}

static int start_chunk(uint64_t addr, uint32_t len, uint32_t buffer)
{
	if (len == 0 || len > 4096 || (len & 15u))
		return -1;
	chunk_dma_buffer_write(buffer);
	chunk_dma_addr_lo_write((uint32_t)addr);
	chunk_dma_addr_hi_write((uint32_t)(addr >> 32));
	chunk_dma_length_write(len / 4);
	chunk_dma_start_write(1);
	return 0;
}

static int scan_buffer(uint32_t buffer, uint64_t addr, uint32_t len,
		       const uint32_t pattern[4], uint64_t *match)
{
	uint32_t off;

	chunk_dma_rd_buffer_write(buffer);
	for (off = 0; off < len; off += 16) {
		uint64_t pair0;
		uint64_t pair1;

		chunk_dma_rd_index_write(off / 8);
		asm volatile("nop; nop; nop; nop;" ::: "memory");
		pair0 = chunk_dma_rd_data_read();
		if ((uint32_t)pair0 != pattern[0])
			continue;

		chunk_dma_rd_index_write(off / 8 + 1);
		asm volatile("nop; nop; nop; nop;" ::: "memory");
		pair1 = chunk_dma_rd_data_read();
		if ((uint32_t)(pair0 >> 32) != pattern[1] ||
		    (uint32_t)pair1 != pattern[2] ||
		    (uint32_t)(pair1 >> 32) != pattern[3])
			continue;

		*match = addr + off;
		return 1;
	}
	return 0;
}

static void peek_buffer(void)
{
	uint32_t i;

	chunk_dma_rd_buffer_write(0);
	for (i = 0; i < 2; i++) {
		uint64_t value;

		chunk_dma_rd_index_write(i);
		asm volatile("nop; nop; nop; nop;" ::: "memory");
		value = chunk_dma_rd_data_read();
		uart_puts("pair");
		uart_putc('0' + i);
		uart_puts("=0x");
		put_hex64(value);
		uart_puts("\r\n");
	}
}

static void scan_memory(uint64_t addr, uint64_t len, const uint32_t pattern[4])
{
	uint64_t scanned = 0;
	uint64_t next_progress = 256ull * 1024 * 1024;
	uint64_t match;
	uint32_t current_len;
	uint32_t next_len = 0;
	uint32_t current_buf = 0;
	uint32_t next_buf = 0;
	uint32_t wait_count = 0;
	int have_next;

	if (!(chunk_dma_status_read() & STATUS_LINK_READY))
		uart_puts("waiting for PCIe link...\r\n");
	while (!(chunk_dma_status_read() & STATUS_LINK_READY)) {
		if ((wait_count++ & 0xfffff) == 0)
			print_status();
	}

	current_len = (len > 4096) ? 4096 : (uint32_t)len;
	if (start_chunk(addr, current_len, current_buf) != 0) {
		uart_puts("error: bad chunk\r\n");
		return;
	}

	for (;;) {
		uint64_t current_addr;

		if (wait_chunk() != 0)
			return;

		current_addr = addr + scanned;
		scanned += current_len;
		have_next = scanned < len;
		if (have_next) {
			next_len = ((len - scanned) > 4096) ? 4096
				: (uint32_t)(len - scanned);
			next_buf = current_buf ^ 1;
			if (start_chunk(addr + scanned, next_len, next_buf) != 0) {
				uart_puts("error: bad chunk\r\n");
				return;
			}
		}

		if (scan_buffer(current_buf, current_addr, current_len,
				pattern, &match)) {
			if (have_next)
				(void)wait_chunk();
			uart_puts("match 0x");
			put_hex64(match);
			uart_puts("\r\n");
			return;
		}

		if (!have_next)
			break;

		if (scanned >= next_progress) {
			uart_puts("scan ");
			put_hex64(scanned);
			uart_puts("\r\n");
			next_progress += 256ull * 1024 * 1024;
		}

		current_buf = next_buf;
		current_len = next_len;
	}

	uart_puts("no match\r\n");
}

int main(void)
{
	uart_puts("\r\nPCIe Screamer Squirrel console\r\n");
	uart_puts("Type 'help' for commands.\r\n\r\n");

	for (;;) {
		char *argv[8];
		int argc;
		int length;

		uart_puts("soc> ");
		length = readline(line_buffer, LINE_MAX);
		if (length < 0) {
			uart_puts("error: line too long\r\n");
			continue;
		}
		if (length == 0)
			continue;

		argc = tokenize(line_buffer, argv, 8);
		if (argc < 0) {
			uart_puts("error: too many arguments\r\n");
			continue;
		}
		if (argc == 0)
			continue;

		if (str_eq(argv[0], "help")) {
			if (argc != 1) {
				uart_puts("error: too many arguments\r\n");
				continue;
			}
			print_help();
		} else if (str_eq(argv[0], "scan")) {
			uint32_t addr_hi;
			uint32_t addr_lo;
			uint32_t len_hi;
			uint32_t len_lo;
			uint64_t addr;
			uint64_t len;
			uint32_t pattern[4];
			int bad = 0;
			int i;

			if (argc != 7) {
				uart_puts("error: scan needs addr, len, and four pattern words\r\n");
				continue;
			}
			if (parse_hex64(argv[1], &addr_hi, &addr_lo) != 0 ||
			    parse_hex64(argv[2], &len_hi, &len_lo) != 0) {
				uart_puts("error: invalid address or length\r\n");
				continue;
			}
			addr = ((uint64_t)addr_hi << 32) | addr_lo;
			len = ((uint64_t)len_hi << 32) | len_lo;
			for (i = 0; i < 4; i++) {
				uint32_t hi;
				uint32_t lo;

				if (parse_hex64(argv[3 + i], &hi, &lo) != 0 || hi != 0) {
					bad = 1;
					break;
				}
				pattern[i] = lo;
			}
			if (bad) {
				uart_puts("error: invalid pattern word\r\n");
				continue;
			}
			if (addr & 15ull) {
				uart_puts("error: address must be 16-byte aligned\r\n");
				continue;
			}
			if (len == 0 || (len & 15ull)) {
				uart_puts("error: length must be nonzero and 16-byte aligned\r\n");
				continue;
			}
			if (addr + len < addr) {
				uart_puts("error: address range overflow\r\n");
				continue;
			}
			scan_memory(addr, len, pattern);
		} else if (str_eq(argv[0], "peek")) {
			if (argc != 1) {
				uart_puts("error: too many arguments\r\n");
				continue;
			}
			peek_buffer();
		} else if (str_eq(argv[0], "status")) {
			if (argc != 1) {
				uart_puts("error: too many arguments\r\n");
				continue;
			}
			print_status();
		} else {
			uart_puts("unknown command: ");
			uart_puts(argv[0]);
			uart_puts("\r\n");
		}
	}
}
