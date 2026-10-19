#include "ax25.h"

#include <ctype.h>
#include <stdio.h>
#include <string.h>

uint16_t ax25_crc(const uint8_t *data, int len)
{
    uint16_t crc = 0xFFFF;
    for (int i = 0; i < len; i++)
    {
        crc ^= data[i];
        for (int b = 0; b < 8; b++)
            crc = (crc & 1) ? (crc >> 1) ^ 0x8408 : (crc >> 1);
    }
    return crc ^ 0xFFFF;
}

/* Parse "CALL[-SSID][*]" up to one of the terminators. Returns chars consumed, or -1. */
static int parse_addr(const char *s, const char *terms, uint8_t addr[7], int *h_bit)
{
    int n = 0;
    memset(addr, ' ' << 1, 6);
    while (s[n] && !strchr(terms, s[n]) && s[n] != '-' && s[n] != '*')
    {
        if (n >= 6 || !isalnum((unsigned char)s[n]))
            return -1;
        addr[n] = (uint8_t)(toupper((unsigned char)s[n]) << 1);
        n++;
    }
    if (n == 0)
        return -1;

    int ssid = 0;
    if (s[n] == '-')
    {
        n++;
        if (!isdigit((unsigned char)s[n]))
            return -1;
        while (isdigit((unsigned char)s[n]))
            ssid = ssid * 10 + (s[n++] - '0');
        if (ssid > 15)
            return -1;
    }
    *h_bit = 0;
    if (s[n] == '*')
    {
        *h_bit = 1;
        n++;
    }
    if (!s[n] || !strchr(terms, s[n]))
        return -1;

    addr[6] = (uint8_t)(0x60 | (ssid << 1));
    return n;
}

int ax25_from_tnc2(const char *tnc2, uint8_t *out, int out_size)
{
    uint8_t addrs[2 + AX25_MAX_REPEATERS][7];
    int h;
    const char *p = tnc2;

    /* source */
    int n = parse_addr(p, ">", addrs[1], &h);
    if (n < 0)
        return -1;
    p += n + 1;

    /* destination */
    n = parse_addr(p, ",:", addrs[0], &h);
    if (n < 0)
        return -1;
    p += n;

    int naddr = 2;
    while (*p == ',')
    {
        if (naddr >= 2 + AX25_MAX_REPEATERS)
            return -1;
        p++;
        n = parse_addr(p, ",:", addrs[naddr], &h);
        if (n < 0)
            return -1;
        if (h)
            addrs[naddr][6] |= 0x80; /* has-been-repeated */
        naddr++;
        p += n;
    }
    if (*p != ':')
        return -1;
    p++;

    int info_len = (int)strlen(p);
    int len = naddr * 7 + 2 + info_len + 2;
    if (info_len > 256 || len > out_size)
        return -1;

    addrs[0][6] |= 0x80; /* command frame: C bit set in destination */
    addrs[naddr - 1][6] |= 0x01; /* address extension: last one */
    for (int i = 0; i < naddr; i++)
        memcpy(out + i * 7, addrs[i], 7);

    int pos = naddr * 7;
    out[pos++] = 0x03; /* UI */
    out[pos++] = 0xF0; /* no layer 3 */
    memcpy(out + pos, p, info_len);
    pos += info_len;

    uint16_t fcs = ax25_crc(out, pos);
    out[pos++] = fcs & 0xFF;
    out[pos++] = fcs >> 8;
    return pos;
}

static int fmt_addr(const uint8_t *a, char *out)
{
    int n = 0;
    for (int i = 0; i < 6; i++)
    {
        char c = (char)(a[i] >> 1);
        if (c != ' ')
            out[n++] = c;
    }
    int ssid = (a[6] >> 1) & 0x0F;
    if (ssid)
        n += sprintf(out + n, "-%d", ssid);
    return n;
}

int ax25_to_tnc2(const uint8_t *frame, int len, char *out, int out_size)
{
    int naddr = 0;
    while (naddr * 7 < len)
    {
        naddr++;
        if (frame[naddr * 7 - 1] & 0x01)
            break;
    }
    if (naddr < 2 || naddr > 2 + AX25_MAX_REPEATERS || !(frame[naddr * 7 - 1] & 0x01) || naddr * 7 + 2 > len)
        return -1;
    if (out_size < 128)
        return -1;

    /* Mark only the last repeater that has the H bit set, as TNC2 does. */
    int last_h = -1;
    for (int i = 2; i < naddr; i++)
        if (frame[i * 7 + 6] & 0x80)
            last_h = i;

    int n = fmt_addr(frame + 7, out);
    out[n++] = '>';
    n += fmt_addr(frame, out + n);
    for (int i = 2; i < naddr; i++)
    {
        out[n++] = ',';
        n += fmt_addr(frame + i * 7, out + n);
        if (i == last_h)
            out[n++] = '*';
    }
    out[n++] = ':';

    int pos = naddr * 7;
    uint8_t ctrl = frame[pos++];
    if (ctrl != 0x03)
    {
        n += snprintf(out + n, out_size - n, "<non-UI ctrl 0x%02X>", ctrl);
        return n;
    }
    pos++; /* PID */

    for (; pos < len && n < out_size - 7; pos++)
    {
        uint8_t c = frame[pos];
        if (c >= 0x20 && c < 0x7F)
            out[n++] = (char)c;
        else
            n += sprintf(out + n, "<0x%02X>", c);
    }
    out[n] = '\0';
    return n;
}
