// SPDX-License-Identifier: GPL-2.0+
/*
 * Copyright (c) 2012, The Chromium Authors
 */

#define DEBUG

#include <common.h>
#if defined(CONFIG_EFI_LOADER) && \
	!defined(CONFIG_SPL_BUILD) && !defined(API_BUILD)
#include <efi_api.h>
#endif
#include <display_options.h>
#include <version.h>

#define FAKE_BUILD_TAG	"jenkins-u-boot-denx_uboot_dm-master-build-aarch64" \
			"and a lot more text to come"

/* Test efi_loader specific printing */
static void efi_ut_print(void)
{
#if defined(CONFIG_EFI_LOADER) && \
    !defined(CONFIG_SPL_BUILD) && !defined(API_BUILD)
	char str[10];
	u8 buf[sizeof(struct efi_device_path_sd_mmc_path) +
	       sizeof(struct efi_device_path)];
	u8 *pos = buf;
	struct efi_device_path *dp_end;
	struct efi_device_path_sd_mmc_path *dp_sd =
			(struct efi_device_path_sd_mmc_path *)pos;

	/* Create a device path for an SD card */
	dp_sd->dp.type = DEVICE_PATH_TYPE_MESSAGING_DEVICE;
	dp_sd->dp.sub_type = DEVICE_PATH_SUB_TYPE_MSG_SD;
	dp_sd->dp.length = sizeof(struct efi_device_path_sd_mmc_path);
	dp_sd->slot_number = 3;
	pos += sizeof(struct efi_device_path_sd_mmc_path);
	/* Append end node */
	dp_end = (struct efi_device_path *)pos;
	dp_end->type = DEVICE_PATH_TYPE_END;
	dp_end->sub_type = DEVICE_PATH_SUB_TYPE_END;
	dp_end->length = sizeof(struct efi_device_path);

	snprintf(str, sizeof(str), "_%pD_", buf);
	assert(!strcmp("_/SD(3)_", str));

	/* NULL device path */
	snprintf(str, sizeof(str), "_%pD_", NULL);
	assert(!strcmp("_<NULL>_", str));
#endif
}

static int do_ut_print(cmd_tbl_t *cmdtp, int flag, int argc,
		       char *const argv[])
{
	char big_str[400];
	int big_str_len;
	char str[10], *s;
	int len;

	printf("%s: Testing print\n", __func__);

	snprintf(str, sizeof(str), "testing");
	assert(!strcmp("testing", str));

	snprintf(str, sizeof(str), "testing but too long");
	assert(!strcmp("testing b", str));

	snprintf(str, 1, "testing none");
	assert(!strcmp("", str));

	*str = 'x';
	snprintf(str, 0, "testing none");
	assert(*str == 'x');

	sprintf(big_str, "_%ls_", L"foo");
	assert(!strcmp("_foo_", big_str));

	/* Test the banner function */
	s = display_options_get_banner(true, str, sizeof(str));
	assert(s == str);
	assert(!strcmp("\n\nU-Boo\n\n", s));

	s = display_options_get_banner(true, str, 1);
	assert(s == str);
	assert(!strcmp("", s));

	s = display_options_get_banner(true, str, 2);
	assert(s == str);
	assert(!strcmp("\n", s));

	s = display_options_get_banner(false, str, sizeof(str));
	assert(s == str);
	assert(!strcmp("U-Boot \n\n", s));

	/* Give it enough space for some of the version */
	big_str_len = strlen(version_string) - 5;
	s = display_options_get_banner_priv(false, FAKE_BUILD_TAG, big_str,
					    big_str_len);
	assert(s == big_str);
	assert(!strncmp(version_string, s, big_str_len - 3));
	assert(!strcmp("\n\n", s + big_str_len - 3));

	/* Give it enough space for the version and some of the build tag */
	big_str_len = strlen(version_string) + 9 + 20;
	s = display_options_get_banner_priv(false, FAKE_BUILD_TAG, big_str,
					    big_str_len);
	assert(s == big_str);
	len = strlen(version_string);
	assert(!strncmp(version_string, s, len));
	assert(!strncmp(", Build: ", s + len, 9));
	assert(!strncmp(FAKE_BUILD_TAG, s + 9 + len, 12));
	assert(!strcmp("\n\n", s + big_str_len - 3));

	/* Test efi_loader specific printing */
	efi_ut_print();

	printf("%s: Everything went swimmingly\n", __func__);
	return 0;
}

U_BOOT_CMD(
	ut_print,	1,	1,	do_ut_print,
	"Very basic test of printf(), etc.",
	""
);
