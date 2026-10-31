#include <stdint.h>

#include <generated/csr.h>
#include <generated/mem.h>

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
	uart_puts("  scan <addr64> <len64> <w0> <w1> <w2> <w3> [stride]\r\n");
	uart_puts("                      stride defaults to 16 and is a 16-byte multiple\r\n");
	uart_puts("  bench <addr64> <len64>  raw 4-outstanding PCIe read benchmark\r\n");
	uart_puts("  peek                show first 16 bytes of DMA buffer 0\r\n");
	uart_puts("  rpeek               show first 32 bytes of ReadBench buffer 0\r\n");
	uart_puts("  wtest               write 0x1122334455667788 to ReadBench word 0\r\n");
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

static uint32_t bswap32(uint32_t v)
{
	return ((v & 0x000000ffu) << 24) |
	       ((v & 0x0000ff00u) << 8) |
	       ((v & 0x00ff0000u) >> 8) |
	       ((v & 0xff000000u) >> 24);
}

static int scan_buffer(uint32_t buffer, uint64_t addr, uint32_t len,
		       const uint32_t pattern[4], uint32_t stride,
		       uint64_t *match)
{
	const volatile uint32_t *buf =
		(const volatile uint32_t *)(READ_BUF_BASE +
					    (buffer * 0x1000u));
	uint32_t off;

	for (off = 0; off + 16u <= len; off += stride) {
		uint32_t w0 = bswap32(buf[off / 4u + 0u]);
		uint32_t w1 = bswap32(buf[off / 4u + 1u]);
		uint32_t w2;
		uint32_t w3;

		if (w0 != pattern[0] || w1 != pattern[1])
			continue;
		w2 = bswap32(buf[off / 4u + 2u]);
		w3 = bswap32(buf[off / 4u + 3u]);
		if (w2 != pattern[2] || w3 != pattern[3])
			continue;

		*match = addr + off;
		return 1;
	}
	return 0;
}



static void peek_read_buffer(void);

static void test_buffer_write(void)
{
	read_bench_test_addr_write(0);
	read_bench_test_data_write(0x1122334455667788ull);
	read_bench_test_we_write(1);
	read_bench_test_we_write(0);
	uart_puts("mem0=0x");
	put_hex64(read_bench_mem0_read());
	uart_puts("\r\n");
	peek_read_buffer();
}

static void peek_read_buffer(void)
{
	const volatile uint32_t *buf =
		(const volatile uint32_t *)READ_BUF_BASE;
	uint32_t i;

	for (i = 0; i < 8; i++) {
		put_hex32(buf[i], 8);
		uart_putc(' ');
	}
	uart_puts("\r\n");
	uart_puts("debug=0x");
	put_hex64(read_bench_debug_read());
	uart_puts("\r\n");
	uart_puts("mem0=0x");
	put_hex64(read_bench_mem0_read());
	uart_puts("\r\n");
	uart_puts("first_addr=0x");
	put_hex32(read_bench_first_addr_read(), 3);
	uart_puts(" first_data=0x");
	put_hex64(read_bench_first_data_read());
	uart_puts(" first_tag=0x");
	put_hex32(read_bench_first_tag_read(), 2);
	uart_puts(" first_wp_we=");
	uart_putc(read_bench_first_wp_we_read() ? '1' : '0');
	uart_puts(" first_test_we=");
	uart_putc(read_bench_first_test_we_read() ? '1' : '0');
	uart_puts("\r\n");
	uart_puts("fifo_seen=");
	uart_putc(read_bench_fifo_seen_read() ? '1' : '0');
	uart_puts(" fifo_tag=0x");
	put_hex32(read_bench_fifo_tag_read(), 1);
	uart_puts(" fifo_addr=0x");
	put_hex32(read_bench_fifo_addr_read(), 3);
	uart_puts(" fifo_data=0x");
	put_hex64(read_bench_fifo_data_read());
	uart_puts(" fifo_wp_we=");
	uart_putc(read_bench_fifo_wp_we_read() ? '1' : '0');
	uart_puts("\r\n");
	uart_puts("cnt_seen=");
	uart_putc(read_bench_cnt_seen_read() ? '1' : '0');
	uart_puts(" cnt_tag=0x");
	put_hex32(read_bench_cnt_tag_read(), 1);
	uart_puts(" cnt_count=0x");
	put_hex32(read_bench_cnt_count_read(), 3);
	uart_puts(" cnt_data=0x");
	put_hex32(read_bench_cnt_data_read(), 3);
	uart_puts(" cnt_len=0x");
	put_hex32(read_bench_cnt_len_read(), 3);
	uart_puts(" cnt_new=0x");
	put_hex32(read_bench_cnt_new_read(), 3);
	uart_puts("\r\n");
}

static void peek_buffer(void)
{
	uint32_t i;

	main_bench_select_write(0);
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


static void bench_memory(uint64_t addr, uint64_t len)
{
	uint32_t wait_count = 0;

	main_bench_select_write(1);
	read_bench_auto_release_write(1);
	read_bench_stop_write(0);
	while (!(read_bench_status_read() & STATUS_LINK_READY)) {
		if ((wait_count++ & 0xfffff) == 0)
			print_status();
	}
	read_bench_addr_lo_write((uint32_t)addr);
	read_bench_addr_hi_write((uint32_t)(addr >> 32));
	read_bench_length_lo_write((uint32_t)len);
	read_bench_length_hi_write((uint32_t)(len >> 32));
	read_bench_start_write(1);
	while (!(read_bench_status_read() & (STATUS_DONE | STATUS_ERROR)))
		;
	if (read_bench_status_read() & STATUS_ERROR) {
		uart_puts("bench error: 0x");
		put_hex32(read_bench_error_read(), 8);
		uart_puts("\r\n");
	} else {
		uart_puts("bench done\r\n");
	}
	main_bench_select_write(0);
}

static void scan_memory(uint64_t addr, uint64_t len, const uint32_t pattern[4],
			uint32_t stride)
{
	uint64_t scanned = 0;
	uint64_t next_progress = 256ull * 1024 * 1024;
	uint64_t match;
	uint32_t tag = 0;
	uint32_t wait_count = 0;
	uint32_t settle;

	main_bench_select_write(1);
	read_bench_auto_release_write(0);
	read_bench_stop_write(0);
	if (!(read_bench_status_read() & STATUS_LINK_READY))
		uart_puts("waiting for PCIe link...\r\n");
	while (!(read_bench_status_read() & STATUS_LINK_READY)) {
		if ((wait_count++ & 0xfffff) == 0)
			print_status();
	}
	read_bench_addr_lo_write((uint32_t)addr);
	read_bench_addr_hi_write((uint32_t)(addr >> 32));
	read_bench_length_lo_write((uint32_t)len);
	read_bench_length_hi_write((uint32_t)(len >> 32));
	read_bench_start_write(1);

	while (scanned < len) {
		uint32_t current_len = ((len - scanned) > 4096ull) ?
			4096u : (uint32_t)(len - scanned);

		while (!(read_bench_tag_ready_read() & (1u << tag))) {
			if (read_bench_status_read() & STATUS_ERROR) {
				uart_puts("error: ring dma 0x");
				put_hex32(read_bench_error_read(), 8);
				uart_puts("\r\n");
				main_bench_select_write(0);
				return;
			}
		}
		for (settle = 0; settle < 16; settle++)
			asm volatile("nop;" ::: "memory");

		if (scan_buffer(tag, addr + scanned, current_len,
				pattern, stride, &match)) {
			read_bench_stop_write(1);
			while (!(read_bench_status_read() & (STATUS_DONE | STATUS_ERROR)))
				;
			main_bench_select_write(0);
			uart_puts("match 0x");
			put_hex64(match);
			uart_puts("\r\n");
			return;
		}

		read_bench_release_write(1u << tag);
		scanned += current_len;
		tag = (tag + 1) & 3;
		if (scanned >= next_progress) {
			uart_puts("scan ");
			put_hex64(scanned);
			uart_puts("\r\n");
			next_progress += 256ull * 1024 * 1024;
		}
	}

	read_bench_stop_write(1);
	while (!(read_bench_status_read() & (STATUS_DONE | STATUS_ERROR)))
		;
	main_bench_select_write(0);
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
			uint32_t stride = 16;
			int bad = 0;
			int i;

			if (argc != 7 && argc != 8) {
				uart_puts("error: scan needs addr, len, four pattern words, and optional stride\r\n");
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
			if (argc == 8) {
				uint32_t stride_hi;
				uint32_t stride_lo;

				if (parse_hex64(argv[7], &stride_hi,
						&stride_lo) != 0 ||
				    stride_hi != 0 || stride_lo == 0 ||
				    (stride_lo & 15u) || stride_lo > 4096u ||
				    stride_lo > len) {
					uart_puts("error: invalid stride\r\n");
					continue;
				}
				stride = stride_lo;
			}
			scan_memory(addr, len, pattern, stride);
		} else if (str_eq(argv[0], "bench")) {
			uint32_t addr_hi;
			uint32_t addr_lo;
			uint32_t len_hi;
			uint32_t len_lo;
			uint64_t addr;
			uint64_t len;

			if (argc != 3) {
				uart_puts("error: bench needs addr and length\r\n");
				continue;
			}
			if (parse_hex64(argv[1], &addr_hi, &addr_lo) != 0 ||
			    parse_hex64(argv[2], &len_hi, &len_lo) != 0) {
				uart_puts("error: invalid address or length\r\n");
				continue;
			}
			addr = ((uint64_t)addr_hi << 32) | addr_lo;
			len = ((uint64_t)len_hi << 32) | len_lo;
			if ((addr & 3ull) || len == 0 || (len & 3ull) ||
			    addr + len < addr) {
				uart_puts("error: invalid bench range\r\n");
				continue;
			}
			bench_memory(addr, len);
		} else if (str_eq(argv[0], "wtest")) {
			if (argc != 1) {
				uart_puts("error: too many arguments\r\n");
				continue;
			}
			test_buffer_write();
		} else if (str_eq(argv[0], "rpeek")) {
			if (argc != 1) {
				uart_puts("error: too many arguments\r\n");
				continue;
			}
			peek_read_buffer();
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
