#include <stdint.h>

#include <generated/csr.h>

#define STATUS_LINK_READY (1u << 0)
#define STATUS_DONE       (1u << 2)
#define STATUS_ERROR      (1u << 3)

#define DEFAULT_ADDR      0x0003f000u
#define DEFAULT_LENGTH    0x00001000u
#define MAX_LENGTH        0x00001000u
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

static void put_hex8(uint32_t value)
{
	put_hex32(value, 2);
}

static void print_status(void)
{
	uart_puts("tlp=0x");
	put_hex32(tlp_dma_status_read(), 2);
	uart_puts(" link=0x");
	put_hex32(pcie_phy_link_status_read(), 8);
	uart_puts(" bme=");
	uart_putc((pcie_phy_bus_master_enable_read() & 1) ? '1' : '0');
	uart_puts(" id=0x");
	put_hex32(main_pcie_id_read(), 4);
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
	uart_puts("  dump [addr] [len]   read physical memory (default 0x3f000 0x1000)\r\n");
	uart_puts("                      addr, len in hex; both 4-byte aligned; len 4..0x1000\r\n");
	uart_puts("WARNING: arbitrary physical reads may hang or crash the host!\r\n");
}

static void dump_memory(uint32_t addr_hi, uint32_t addr_lo, uint32_t length)
{
	uint32_t status;
	uint32_t offset;
	uint32_t lane;
	uint32_t wait_count = 0;

	if (!(tlp_dma_status_read() & STATUS_LINK_READY))
		uart_puts("waiting for PCIe link...\r\n");
	while (!(tlp_dma_status_read() & STATUS_LINK_READY)) {
		if ((wait_count++ & 0xfffff) == 0)
			print_status();
	}

	tlp_dma_addr_lo_write(addr_lo);
	tlp_dma_addr_hi_write(addr_hi);
	tlp_dma_length_write(length / 4);
	tlp_dma_start_write(1);

	do {
		status = tlp_dma_status_read();
	} while (!(status & (STATUS_DONE | STATUS_ERROR)));

	if (status & STATUS_ERROR) {
		uart_puts("tlp dma error: 0x");
		put_hex32(tlp_dma_error_read(), 8);
		uart_puts("\r\n");
		return;
	}

	for (offset = 0; offset < length; offset += 16) {
		uart_puts("0x");
		put_hex32(addr_hi, 8);
		put_hex32(addr_lo, 8);
		uart_puts(": ");

		for (lane = 0; lane < 16 && (offset + lane) < length; lane += 4) {
			uint32_t data;

			tlp_dma_rd_index_write((offset + lane) / 4);
			asm volatile("nop; nop; nop; nop;" ::: "memory");
			data = tlp_dma_rd_data_read();

			put_hex8((data >> 24) & 0xff);
			uart_putc(' ');
			put_hex8((data >> 16) & 0xff);
			uart_putc(' ');
			put_hex8((data >> 8) & 0xff);
			uart_putc(' ');
			put_hex8(data & 0xff);
			uart_putc(' ');
		}
		uart_puts("\r\n");

		addr_lo += 16;
		if (addr_lo < 16)
			addr_hi++;
	}
}

int main(void)
{
	uart_puts("\r\nPCIe Screamer Squirrel console\r\n");
	uart_puts("Type 'help' for commands.\r\n\r\n");

	for (;;) {
		char *argv[3];
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

		argc = tokenize(line_buffer, argv, 3);
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
		} else if (str_eq(argv[0], "status")) {
			if (argc != 1) {
				uart_puts("error: too many arguments\r\n");
				continue;
			}
			print_status();
		} else if (str_eq(argv[0], "dump")) {
			uint32_t addr_hi = 0;
			uint32_t addr_lo = DEFAULT_ADDR;
			uint32_t len = DEFAULT_LENGTH;

			if (argc >= 2 && parse_hex64(argv[1], &addr_hi, &addr_lo) != 0) {
				uart_puts("error: invalid address\r\n");
				continue;
			}
			if (argc >= 3) {
				uint32_t len_hi;

				if (parse_hex64(argv[2], &len_hi, &len) != 0 || len_hi != 0) {
					uart_puts("error: invalid length\r\n");
					continue;
				}
			}
			if (addr_lo & 3u) {
				uart_puts("error: address must be 4-byte aligned\r\n");
				continue;
			}
			if (len == 0 || len > MAX_LENGTH || (len & 3u)) {
				uart_puts("error: length must be 4..0x1000 and 4-byte aligned\r\n");
				continue;
			}
			dump_memory(addr_hi, addr_lo, len);
		} else {
			uart_puts("unknown command: ");
			uart_puts(argv[0]);
			uart_puts("\r\n");
		}
	}
}
