#include <stdint.h>

#include <generated/csr.h>

#define SYNC_WORD          0x66665555u
#define TLP_FLAG_MAGIC     0x77000000u
#define TLP_FLAG_LAST      (1u << 18)

#define CPLD_FMT_TYPE      0x4au
#define CPL_FMT_TYPE       0x0au

#define BRIDGE_LINK_READY  (1u << 0)

#define DEFAULT_ADDR       0x0003f000u
#define DEFAULT_LENGTH     0x00001000u
#define MAX_LENGTH         0x00001000u
#define LINE_MAX           128

#define RX_TIMEOUT_COUNT   20000000u
#define RX_WORD_BUDGET     65536u

#define RX_OK              0
#define RX_TIMEOUT        -1
#define RX_UNEXPECTED     -2
#define RX_MALFORMED      -3
#define RX_STATUS         -4
#define RX_TAG            -5
#define RX_ID             -6
#define RX_BYTE_COUNT     -7
#define RX_ADDR           -8

void isr(void);
int main(void);

static char line_buffer[LINE_MAX];
static uint8_t mrd_tag;

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

static void bridge_write(uint32_t word)
{
	while (pcileech_bridge_tx_full_read())
		;
	pcileech_bridge_tx_data_write(word);
}

static uint32_t bridge_read(void)
{
	while (pcileech_bridge_rx_empty_read())
		;
	return pcileech_bridge_rx_data_read();
}

static int bridge_read_timeout(uint32_t *word)
{
	uint32_t timeout = RX_TIMEOUT_COUNT;

	while (pcileech_bridge_rx_empty_read()) {
		if (--timeout == 0)
			return -1;
	}
	*word = bridge_read();
	return 0;
}

static void drain_rx(void)
{
	while (!pcileech_bridge_rx_empty_read())
		(void)pcileech_bridge_rx_data_read();
}

static void print_status(void)
{
	uint32_t bridge_status = pcileech_bridge_status_read();

	uart_puts("bridge link=");
	uart_putc((bridge_status & BRIDGE_LINK_READY) ? '1' : '0');
	uart_puts(" tx_full=");
	uart_putc(pcileech_bridge_tx_full_read() ? '1' : '0');
	uart_puts(" rx_empty=");
	uart_putc(pcileech_bridge_rx_empty_read() ? '1' : '0');
	uart_puts("\r\n");
	uart_puts("pcie link=0x");
	put_hex32(pcie_phy_link_status_read(), 8);
	uart_puts(" bme=");
	uart_putc((pcie_phy_bus_master_enable_read() & 1u) ? '1' : '0');
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
	uart_puts("  status              show PCIe/bridge status\r\n");
	uart_puts("  dump [addr] [len]   read physical memory (default 0x3f000 0x1000)\r\n");
	uart_puts("                      addr, len in hex; both 4-byte aligned; len 4..0x1000\r\n");
	uart_puts("WARNING: arbitrary physical reads may hang or crash the host!\r\n");
}

static void send_mrd(uint32_t addr_hi, uint32_t addr_lo, uint8_t tag)
{
	uint32_t dw1 = (main_pcie_id_read() << 16) | ((uint32_t)tag << 8) | 0x000fu;

	if (addr_hi == 0) {
		bridge_write(0x00000001u);
		bridge_write(TLP_FLAG_MAGIC);
		bridge_write(dw1);
		bridge_write(TLP_FLAG_MAGIC);
		bridge_write(addr_lo);
		bridge_write(TLP_FLAG_MAGIC | TLP_FLAG_LAST);
	} else {
		bridge_write(0x20000001u);
		bridge_write(TLP_FLAG_MAGIC);
		bridge_write(dw1);
		bridge_write(TLP_FLAG_MAGIC);
		bridge_write(addr_hi);
		bridge_write(TLP_FLAG_MAGIC);
		bridge_write(addr_lo);
		bridge_write(TLP_FLAG_MAGIC | TLP_FLAG_LAST);
	}
}

static int read_cpld(uint32_t *data, uint32_t *error, uint32_t addr_lo, uint8_t tag)
{
	static const uint8_t ctx_shift[7] = {24, 28, 16, 20, 8, 12, 0};
	uint32_t cpl[4];
	uint32_t header = 0;
	uint32_t id = main_pcie_id_read();
	uint32_t budget = RX_WORD_BUDGET;
	int count = 0;
	int need = 4;
	int frame_pos = 0;

	while (budget != 0) {
		uint32_t word;
		uint32_t ctx;
		int last;
		int index;

		budget--;
		if (bridge_read_timeout(&word) != 0)
			return RX_TIMEOUT;

		if (frame_pos == 0 && word == SYNC_WORD) {
			count = 0;
			need = 4;
			frame_pos = 0;
			continue;
		}

		if (frame_pos == 0) {
			if (((word >> 4) & 0xfu) != 0xeu) {
				count = 0;
				need = 4;
				continue;
			}
			header = word;
			frame_pos = 1;
			continue;
		}

		index = frame_pos - 1;
		ctx = (header >> ctx_shift[index]) & 0xfu;
		frame_pos++;
		if (frame_pos == 8)
			frame_pos = 0;

		if (ctx == 0xfu)
			continue;
		if ((ctx & 0x3u) != 0)
			continue;

		last = (ctx & 0x4u) != 0;

		if (last && count != need - 1)
			return RX_MALFORMED;

		if (count == 0) {
			if ((word >> 24) == CPLD_FMT_TYPE) {
				cpl[0] = word;
				count = 1;
				need = 4;
			} else if ((word >> 24) == CPL_FMT_TYPE) {
				cpl[0] = word;
				count = 1;
				need = 3;
			} else {
				return RX_UNEXPECTED;
			}
			continue;
		}

		cpl[count++] = word;
		if (count != need)
			continue;

		if (!last)
			return RX_MALFORMED;

		if (need == 3) {
			if (((cpl[2] >> 8) & 0xffu) != tag)
				return RX_TAG;
			if ((cpl[2] >> 16) != id)
				return RX_ID;
			*error = (cpl[1] >> 13) & 0x7u;
			if (*error == 0)
				return RX_MALFORMED;
			return RX_STATUS;
		}

		if ((cpl[0] & 0x3ffu) != 1u)
			return RX_MALFORMED;
		if ((cpl[1] & 0xfffu) != 4u)
			return RX_BYTE_COUNT;
		if (((cpl[2] >> 8) & 0xffu) != tag)
			return RX_TAG;
		if ((cpl[2] >> 16) != id)
			return RX_ID;
		if ((cpl[2] & 0x7fu) != (addr_lo & 0x7fu))
			return RX_ADDR;
		*error = (cpl[1] >> 13) & 0x7u;
		if (*error != 0)
			return RX_STATUS;
		*data = cpl[3];
		return RX_OK;
	}

	return RX_TIMEOUT;
}

static void print_rx_error(int result, uint32_t error)
{
	if (result == RX_TIMEOUT)
		uart_puts("error: completion timeout\r\n");
	else if (result == RX_UNEXPECTED)
		uart_puts("error: unexpected TLP\r\n");
	else if (result == RX_MALFORMED)
		uart_puts("error: malformed completion\r\n");
	else if (result == RX_STATUS) {
		uart_puts("error: completion status=0x");
		put_hex8(error);
		uart_puts("\r\n");
	} else if (result == RX_TAG)
		uart_puts("error: completion tag mismatch\r\n");
	else if (result == RX_ID)
		uart_puts("error: completion requester mismatch\r\n");
	else if (result == RX_BYTE_COUNT)
		uart_puts("error: completion byte count mismatch\r\n");
	else if (result == RX_ADDR)
		uart_puts("error: completion address mismatch\r\n");
	else
		uart_puts("error: completion failed\r\n");
}

static void dump_memory(uint32_t addr_hi, uint32_t addr_lo, uint32_t length)
{
	uint32_t line[4];
	uint32_t offset;
	uint32_t wait_count = 0;
	uint32_t cur_hi = addr_hi;
	uint32_t cur_lo = addr_lo;

	if (!(pcileech_bridge_status_read() & BRIDGE_LINK_READY))
		uart_puts("waiting for PCIe link...\r\n");
	while (!(pcileech_bridge_status_read() & BRIDGE_LINK_READY)) {
		if ((wait_count++ & 0xfffff) == 0)
			print_status();
	}
	drain_rx();

	for (offset = 0; offset < length; offset += 16) {
		uint32_t line_hi = cur_hi;
		uint32_t line_lo = cur_lo;
		uint32_t remaining = length - offset;
		int lanes = remaining >= 16 ? 4 : (int)(remaining / 4);
		int lane;

		for (lane = 0; lane < lanes; lane++) {
			uint32_t error = 0;
			uint8_t tag;
			int result;

			if (!(pcileech_bridge_status_read() & BRIDGE_LINK_READY)) {
				uart_puts("error: PCIe link lost\r\n");
				return;
			}
			tag = mrd_tag++;
			send_mrd(cur_hi, cur_lo, tag);
			result = read_cpld(&line[lane], &error, cur_lo, tag);
			if (result != RX_OK) {
				print_rx_error(result, error);
				return;
			}

			cur_lo += 4;
			if (cur_lo < 4)
				cur_hi++;
		}

		uart_puts("0x");
		put_hex32(line_hi, 8);
		put_hex32(line_lo, 8);
		uart_puts(": ");

		for (lane = 0; lane < lanes; lane++) {
			uint32_t data = line[lane];

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
	}
}

int main(void)
{
	uart_puts("\r\nPCIe Screamer PCILeech console\r\n");
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
