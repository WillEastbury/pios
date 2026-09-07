#include <stdio.h>
#include "usb.h"

static int failures;

static void expect_true(const char *name, bool value)
{
    if (!value) {
        printf("FAIL %s\n", name);
        failures++;
    }
}

static u32 build_valid(u8 *buf)
{
    u32 n = 0;
    buf[n++] = 9; buf[n++] = 2; buf[n++] = 32; buf[n++] = 0;
    buf[n++] = 1; buf[n++] = 1; buf[n++] = 0; buf[n++] = 0x80; buf[n++] = 50;
    buf[n++] = 9; buf[n++] = 4; buf[n++] = 2; buf[n++] = 0; buf[n++] = 2;
    buf[n++] = 8; buf[n++] = 6; buf[n++] = 0; buf[n++] = 0;
    buf[n++] = 7; buf[n++] = 5; buf[n++] = 0x81; buf[n++] = 2;
    buf[n++] = 0x40; buf[n++] = 0; buf[n++] = 1;
    buf[n++] = 7; buf[n++] = 5; buf[n++] = 2; buf[n++] = 2;
    buf[n++] = 0x40; buf[n++] = 0; buf[n++] = 1;
    return n;
}

int main(void)
{
    u8 buf[64] = {0};
    struct usb_device dev = {0};
    u32 len = build_valid(buf);

    expect_true("valid configuration parses",
                usb_parse_config_descriptors(buf, len, &dev));
    expect_true("configuration value", dev.config_value == 1);
    expect_true("interface count", dev.num_interfaces == 1);
    expect_true("endpoint count", dev.num_eps == 2);
    expect_true("interface context", dev.eps[0].iface_class == 8 &&
                dev.eps[0].iface_subclass == 6 &&
                dev.eps[0].iface_protocol == 0);
    expect_true("endpoint direction", dev.eps[0].address == 0x81 &&
                dev.eps[1].address == 2);
    expect_true("endpoint packet size", dev.eps[0].max_packet == 64);

    expect_true("null buffer rejected",
                !usb_parse_config_descriptors(NULL, len, &dev));
    expect_true("short blob rejected",
                !usb_parse_config_descriptors(buf, 8, &dev));
    buf[0] = 0;
    expect_true("zero descriptor length rejected",
                !usb_parse_config_descriptors(buf, len, &dev));
    len = build_valid(buf);
    buf[1] = 3;
    expect_true("wrong configuration type rejected",
                !usb_parse_config_descriptors(buf, len, &dev));
    len = build_valid(buf);
    buf[2] = 0x40;
    expect_true("declared total beyond blob rejected",
                !usb_parse_config_descriptors(buf, len, &dev));
    len = build_valid(buf);
    buf[9] = 7;
    buf[10] = 5;
    expect_true("endpoint before interface rejected",
                !usb_parse_config_descriptors(buf, len, &dev));

    if (failures) {
        printf("usb descriptor: %d failures\n", failures);
        return 1;
    }
    printf("usb descriptor: all checks passed\n");
    return 0;
}
