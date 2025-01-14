/*
 * Copyright (c) 2017 BayLibre, SAS
 * Copyright (c) 2020 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <errno.h>
#include <string.h>

#include <zephyr/zephyr.h>
#include <zephyr/device.h>
#include <stdio.h>
#include <zephyr/sys/util.h>

#include <zephyr/drivers/i2c.h>
#include <zephyr/drivers/i2c/target/eeprom.h>

#include <zephyr/ztest.h>

#define NODE_EP0 DT_NODELABEL(eeprom0)
#define NODE_EP1 DT_NODELABEL(eeprom1)

#define TEST_DATA_SIZE	20
static const uint8_t eeprom_0_data[TEST_DATA_SIZE] = "0123456789abcdefghij";
static const uint8_t eeprom_1_data[TEST_DATA_SIZE] = "jihgfedcba9876543210";
static uint8_t i2c_buffer[TEST_DATA_SIZE];

/*
 * We need 5x(buffer size) + 1 to print a comma-separated list of each
 * byte in hex, plus a null.
 */
uint8_t buffer_print_eeprom[TEST_DATA_SIZE * 5 + 1];
uint8_t buffer_print_i2c[TEST_DATA_SIZE * 5 + 1];

static void to_display_format(const uint8_t *src, size_t size, char *dst)
{
	size_t i;

	for (i = 0; i < size; i++) {
		sprintf(dst + 5 * i, "0x%02x,", src[i]);
	}
}

static int run_full_read(const struct device *i2c, uint8_t addr,
			 const uint8_t *comp_buffer)
{
	int ret;

	TC_PRINT("Testing full read: Master: %s, address: 0x%x\n",
		 i2c->name, addr);

	/* Read EEPROM from I2C Master requests, then compare */
	ret = i2c_burst_read(i2c, addr, 0, i2c_buffer, TEST_DATA_SIZE);
	zassert_equal(ret, 0, "Failed to read EEPROM");

	if (memcmp(i2c_buffer, comp_buffer, TEST_DATA_SIZE)) {
		to_display_format(i2c_buffer, TEST_DATA_SIZE,
				  buffer_print_i2c);
		to_display_format(comp_buffer, TEST_DATA_SIZE,
				  buffer_print_eeprom);
		TC_PRINT("Error: Buffer contents are different: %s\n",
			 buffer_print_i2c);
		TC_PRINT("                         vs expected: %s\n",
			 buffer_print_eeprom);
		return -EIO;
	}

	return 0;
}

static int run_partial_read(const struct device *i2c, uint8_t addr,
			    const uint8_t *comp_buffer, unsigned int offset)
{
	int ret;

	TC_PRINT("Testing partial read. Master: %s, address: 0x%x, off=%d\n",
		 i2c->name, addr, offset);

	ret = i2c_burst_read(i2c, addr,
			     offset, i2c_buffer, TEST_DATA_SIZE-offset);
	zassert_equal(ret, 0, "Failed to read EEPROM");

	if (memcmp(i2c_buffer, &comp_buffer[offset], TEST_DATA_SIZE-offset)) {
		to_display_format(i2c_buffer, TEST_DATA_SIZE-offset,
				  buffer_print_i2c);
		to_display_format(&comp_buffer[offset], TEST_DATA_SIZE-offset,
				  buffer_print_eeprom);
		TC_PRINT("Error: Buffer contents are different: %s\n",
			 buffer_print_i2c);
		TC_PRINT("                         vs expected: %s\n",
			 buffer_print_eeprom);
		return -EIO;
	}

	return 0;
}

static int run_program_read(const struct device *i2c, uint8_t addr,
			    unsigned int offset)
{
	int ret, i;

	TC_PRINT("Testing program. Master: %s, address: 0x%x, off=%d\n",
		i2c->name, addr, offset);

	for (i = 0 ; i < TEST_DATA_SIZE-offset ; ++i) {
		i2c_buffer[i] = i;
	}

	ret = i2c_burst_write(i2c, addr,
			      offset, i2c_buffer, TEST_DATA_SIZE-offset);
	zassert_equal(ret, 0, "Failed to write EEPROM");

	(void)memset(i2c_buffer, 0xFF, TEST_DATA_SIZE);

	/* Read back EEPROM from I2C Master requests, then compare */
	ret = i2c_burst_read(i2c, addr,
			     offset, i2c_buffer, TEST_DATA_SIZE-offset);
	zassert_equal(ret, 0, "Failed to read EEPROM");

	for (i = 0 ; i < TEST_DATA_SIZE-offset ; ++i) {
		if (i2c_buffer[i] != i) {
			to_display_format(i2c_buffer, TEST_DATA_SIZE-offset,
					  buffer_print_i2c);
			TC_PRINT("Error: Unexpected buffer content: %s\n",
				 buffer_print_i2c);
			return -EIO;
		}
	}

	return 0;
}

ZTEST(i2c_eeprom_target, test_eeprom_target)
{
	const struct device *const eeprom_0 = DEVICE_DT_GET(NODE_EP0);
	const struct device *const i2c_0 = DEVICE_DT_GET(DT_BUS(NODE_EP0));
	int addr_0 = DT_REG_ADDR(NODE_EP0);
	const struct device *const eeprom_1 = DEVICE_DT_GET(NODE_EP1);
	const struct device *const i2c_1 = DEVICE_DT_GET(DT_BUS(NODE_EP1));
	int addr_1 = DT_REG_ADDR(NODE_EP1);
	int ret, offset;

	zassert_not_null(i2c_0, "EEPROM 0 - I2C bus not found");
	zassert_not_null(eeprom_0, "EEPROM 0 device not found");

	zassert_true(device_is_ready(i2c_0), "EEPROM 0 - I2C bus not ready");

	TC_PRINT("Found EEPROM 0 on I2C bus device %s at addr %02x\n",
		 i2c_0->name, addr_0);

	zassert_not_null(i2c_1, "EEPROM 1 - I2C device not found");
	zassert_not_null(eeprom_1, "EEPROM 1 device not found");

	zassert_true(device_is_ready(i2c_1), "EEPROM 1 - I2C bus not ready");

	TC_PRINT("Found EEPROM 1 on I2C bus device %s at addr %02x\n",
		 i2c_1->name, addr_1);

	if (IS_ENABLED(CONFIG_APP_DUAL_ROLE_I2C)) {
		TC_PRINT("Testing dual-role\n");
	} else {
		TC_PRINT("Testing single-role\n");
	}

	/* Program differentiable data into the two devices through a back door
	 * that doesn't use I2C.
	 */
	ret = eeprom_target_program(eeprom_0, eeprom_0_data, TEST_DATA_SIZE);
	zassert_equal(ret, 0, "Failed to program EEPROM 0");
	if (IS_ENABLED(CONFIG_APP_DUAL_ROLE_I2C)) {
		ret = eeprom_target_program(eeprom_1, eeprom_1_data,
					   TEST_DATA_SIZE);
		zassert_equal(ret, 0, "Failed to program EEPROM 1");
	}

	/* Attach each EEPROM to its owning bus as a target device. */
	ret = i2c_target_driver_register(eeprom_0);
	zassert_equal(ret, 0, "Failed to register EEPROM 0");

	if (IS_ENABLED(CONFIG_APP_DUAL_ROLE_I2C)) {
		ret = i2c_target_driver_register(eeprom_1);
		zassert_equal(ret, 0, "Failed to register EEPROM 1");
	}

	/* The simulated EP0 is configured to be accessed as a target device
	 * at addr_0 on i2c_0 and should expose eeprom_0_data.  The validation
	 * uses i2c_1 as a bus master to access this device, which works because
	 * i2c_0 and i2_c have their SDA (SCL) pins shorted (they are on the
	 * same physical bus).  Thus in these calls i2c_1 is a master device
	 * operating on the target address addr_0.
	 *
	 * Similarly validation of EP1 uses i2c_0 as a master with addr_1 and
	 * eeprom_1_data for validation.
	 */
	ret = run_full_read(i2c_1, addr_0, eeprom_0_data);
	zassert_equal(ret, 0,
		     "Full I2C read from EP0 failed");
	if (IS_ENABLED(CONFIG_APP_DUAL_ROLE_I2C)) {
		ret = run_full_read(i2c_0, addr_1, eeprom_1_data);
		zassert_equal(ret, 0,
			      "Full I2C read from EP1 failed");
	}

	for (offset = 0 ; offset < TEST_DATA_SIZE-1 ; ++offset) {
		zassert_equal(0, run_partial_read(i2c_1, addr_0,
			      eeprom_0_data, offset),
			      "Partial I2C read EP0 failed");
		if (IS_ENABLED(CONFIG_APP_DUAL_ROLE_I2C)) {
			zassert_equal(0, run_partial_read(i2c_0, addr_1,
							  eeprom_1_data,
							  offset),
				      "Partial I2C read EP1 failed");
		}
	}

	for (offset = 0 ; offset < TEST_DATA_SIZE-1 ; ++offset) {
		zassert_equal(0, run_program_read(i2c_1, addr_0, offset),
			      "Program I2C read EP0 failed");
		if (IS_ENABLED(CONFIG_APP_DUAL_ROLE_I2C)) {
			zassert_equal(0, run_program_read(i2c_0, addr_1,
							  offset),
				      "Program I2C read EP1 failed");
		}
	}

	/* Detach EEPROM */
	ret = i2c_target_driver_unregister(eeprom_0);
	zassert_equal(ret, 0, "Failed to unregister EEPROM 0");

	if (IS_ENABLED(CONFIG_APP_DUAL_ROLE_I2C)) {
		ret = i2c_target_driver_unregister(eeprom_1);
		zassert_equal(ret, 0, "Failed to unregister EEPROM 1");
	}
}

ZTEST_SUITE(i2c_eeprom_target, NULL, NULL, NULL, NULL, NULL);
