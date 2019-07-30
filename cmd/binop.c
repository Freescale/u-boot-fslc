// SPDX-License-Identifier: GPL-2.0+

#include <common.h>
#include <command.h>
#include <malloc.h>
#include <mapmem.h>
#include <linux/ctype.h>

enum {
	OP_ID_XOR,
	OP_ID_AND,
	OP_ID_OR,
};

void write_to_env_var(char *varname, u8 *result, ulong len)
{
	char *str_output;
	char *str_ptr;
	int i;

	str_output = malloc(len * 2 + 1);
	str_ptr = str_output;

	for (i = 0; i < len; i++) {
		sprintf(str_ptr, "%02x", result[i]);
		str_ptr += 2;
	}
	*str_ptr = '\0';
	setenv(varname, str_output);

	free(str_output);
}

void decode_hexstring(char *hexstr, u8 *result)
{
	int i;
	int acc = 0;

	for (i = 0; i < strlen(hexstr); ++i) {
		char d = hexstr[i];
		int value;

		if (isdigit(d))
			value = (d - '0');
		else
			value = (islower(d) ? toupper(d) : d) - 'A' + 10;

		if (i % 2 == 0) {
			acc = value * 16;
		} else {
			result[i / 2] = acc + value;
			acc = 0;
		}
	}
}

void read_from_env_var(char *varname, u8 *result)
{
	char *str_value;

	str_value = getenv(varname);
	if (str_value)
		decode_hexstring(str_value, result);
	else
		decode_hexstring(varname, result);
}

void read_from_mem(ulong addr, u8 *result, ulong len)
{
	u8 *src;

	src = map_sysmem(addr, len);
	memcpy(result, src, len);
	unmap_sysmem(src);
}

void write_to_mem(char *varname, u8 *result, ulong len)
{
	ulong addr;
	u8 *buf;

	addr = simple_strtoul(varname, NULL, 16);
	buf = map_sysmem(addr, len);
	memcpy(buf, result, len);
	unmap_sysmem(buf);
}

static int do_binop(cmd_tbl_t *cmdtp, int flag, int argc, char * const argv[])
{
	ulong len;
	u8 *result, *src1, *src2;
	char *oparg, *lenarg, *src1arg, *src2arg, *destarg;
	int i, op;

	if (argc < 5)
		return CMD_RET_USAGE;

	oparg = argv[1];
	lenarg = argv[2];
	src1arg = argv[3];
	src2arg = argv[4];

	if (!strcmp(oparg, "xor"))
		op = OP_ID_XOR;
	else if (!strcmp(oparg, "or"))
		op = OP_ID_OR;
	else if (!strcmp(oparg, "and"))
		op = OP_ID_AND;
	else
		return CMD_RET_USAGE;

	len = simple_strtoul(lenarg, NULL, 10);

	src1 = malloc(len);
	src2 = malloc(len);

	if (*src1arg == '*')
		read_from_mem(simple_strtoul(src1arg + 1, NULL, 16), src1, len);
	else
		read_from_env_var(src1arg, src1);

	if (*src2arg == '*')
		read_from_mem(simple_strtoul(src2arg + 1, NULL, 16), src2, len);
	else
		read_from_env_var(src2arg, src2);

	result = malloc(len);

	switch (op) {
	case OP_ID_XOR:
		for (i = 0; i < len; i++)
			result[i] = src1[i] ^ src2[i];
		break;
	case OP_ID_OR:
		for (i = 0; i < len; i++)
			result[i] = src1[i] | src2[i];
		break;
	case OP_ID_AND:
		for (i = 0; i < len; i++)
			result[i] = src1[i] & src2[i];
		break;
	}

	if (argc == 5) {
		for (i = 0; i < len; i++) {
			printf("%02x ", result[i]);
			if (i % 16 == 15)
				puts("\n");
		}
		puts("\n");

		goto exit;
	}

	destarg = argv[5];

	if (*destarg == '*')
		write_to_mem(destarg + 1, result, len); /* Skip asterisk */
	else
		write_to_env_var(destarg, result, len);
exit:
	free(result);
	free(src2);
	free(src1);

	return 0;
}

U_BOOT_CMD(
	binop,	6,	1,	do_binop,
	"compute binary operation",
	"op count [*]src1 [*]src2 [[*]dest]\n"
		"    - compute binary operation of data at/in src1 and\n      src2 (either *memaddr, env var name or hex string)\n      and store result in/at dest, where op is one of\n      xor, or, and."
);
