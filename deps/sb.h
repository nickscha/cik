/* sb.h - v0.2 - public domain data structures - nickscha 2025

A C89 standard compliant, single header, nostdlib (no C Standard Library) String Builder (SB).

LICENSE

  Placed in the public domain and also MIT licensed.
  See end of file for detailed license information.

*/
#ifndef SB_H
#define SB_H

/* #############################################################################
 * # COMPILER SETTINGS
 * #############################################################################
 */
/* Check if using C99 or later (inline is supported) */
#if __STDC_VERSION__ >= 199901L
#define SB_INLINE inline
#define SB_API static
#elif defined(__GNUC__) || defined(__clang__)
#define SB_INLINE __inline__
#define SB_API static
#elif defined(_MSC_VER)
#define SB_INLINE __inline
#define SB_API static
#else
#define SB_INLINE
#define SB_API static
#endif

typedef struct sb
{
  char *buf; /* Pointer to string buffer */
  int cap;   /* Capacity of buffer */
  int len;   /* Current length of content */
  int ovr;   /* Overflow flag (1 if exceeded capacity) */

} sb;

typedef enum sb_pad_mode
{
  SB_PAD_NONE = 0, /* No padding (default) */
  SB_PAD_LEFT,     /* Pad on the left */
  SB_PAD_RIGHT     /* Pad on the right */

} sb_pad_mode;

static unsigned long SB_LUT_POW10[10] = {
    1ul,
    10ul,
    100ul,
    1000ul,
    10000ul,
    100000ul,
    1000000ul,
    10000000ul,
    100000000ul,
    1000000000ul};

static char SB_LUT_DIGITS_2[] =
    "00010203040506070809"
    "10111213141516171819"
    "20212223242526272829"
    "30313233343536373839"
    "40414243444546474849"
    "50515253545556575859"
    "60616263646566676869"
    "70717273747576777879"
    "80818283848586878889"
    "90919293949596979899";

static char SB_SPACES_64[] = "                                                                ";

SB_API SB_INLINE unsigned long sb_pow10u(int p)
{
  if (p < 0)
  {
    p = 0;
  }

  if (p > 9)
  {
    p = 9;
  }

  return SB_LUT_POW10[p];
}

SB_API SB_INLINE void sb_init(sb *sb, char *buffer, int capacity)
{
  sb->buf = buffer;
  sb->cap = (capacity > 0) ? capacity : 0;
  sb->len = 0;
  sb->ovr = 0;

  if (sb->cap > 0)
  {
    sb->buf[0] = '\0';
  }
}

SB_API SB_INLINE void sb_term(sb *sb)
{
  if (sb->cap == 0)
  {
    return;
  }

  if (sb->len < sb->cap)
  {
    sb->buf[sb->len] = '\0';
  }
  else
  {
    sb->buf[sb->cap - 1] = '\0';
    sb->ovr = 1;
    sb->len = sb->cap - 1;
  }
}

SB_API SB_INLINE void sb_putc(sb *sb, char c)
{
  if (sb->len < sb->cap)
  {
    sb->buf[sb->len] = c;
  }
  else
  {
    sb->ovr = 1;
  }
  sb->len++;
}

SB_API SB_INLINE void sb_append_bytes(sb *sb, char *src, int len)
{
  int space = sb->cap - sb->len;

  if (space > 0)
  {
    int copy = (len < space) ? len : space;
    int i;
    for (i = 0; i < copy; ++i)
    {
      sb->buf[sb->len + i] = src[i];
    }
  }

  sb->len += len;

  if (len > space)
  {
    sb->ovr = 1;
  }
}

SB_API SB_INLINE int sb_append_cstr(sb *sb, char *s)
{
  int n = 0;

  while (s[n] != '\0')
  {
    n++;
  }

  sb_append_bytes(sb, s, n);

  return n;
}

SB_API SB_INLINE void sb_append_spaces(sb *sb, int count)
{
  while (count >= 64)
  {
    sb_append_bytes(sb, SB_SPACES_64, 64);
    count -= 64;
  }

  if (count > 0)
  {
    sb_append_bytes(sb, SB_SPACES_64, count);
  }
}

SB_API SB_INLINE int sb_count_digits_ulong(unsigned long v)
{
  int d = 0;

  if (v == 0ul)
  {
    return 1;
  }

  while (v >= 100ul)
  {
    v /= 100ul;
    d += 2;
  }

  return d + (v < 10ul ? 1 : 2);
}

SB_API SB_INLINE int sb_append_ulong_direct(sb *sb, unsigned long v)
{
  char pair[2];
  int digits;
  int written = 0;

  if (v == 0ul)
  {
    sb_putc(sb, '0');
    return 1;
  }

  digits = sb_count_digits_ulong(v);

  /* If number of digits is odd, write the leading single digit. */
  if (digits & 1)
  {
    int leading;
    unsigned long pow_lead;

    pow_lead = sb_pow10u(digits - 1);
    leading = (int)(v / pow_lead);
    sb_putc(sb, (char)('0' + leading));
    v -= (unsigned long)leading * pow_lead;
    written += 1;
    digits -= 1; /* now even */
  }

  if (digits > 0)
  {
    unsigned long div;

    div = sb_pow10u(digits - 2);

    while (digits > 0)
    {
      unsigned long q = v / div; /* 0..99 */
      unsigned int idx = (unsigned int)q * 2u;
      pair[0] = SB_LUT_DIGITS_2[idx];
      pair[1] = SB_LUT_DIGITS_2[idx + 1];

      sb_append_bytes(sb, pair, 2);

      written += 2;
      v -= q * div;

      digits -= 2;

      if (div >= 100ul)
      {
        div /= 100ul;
      }
      else
      {
        break;
      }
    }
  }

  return written;
}

SB_API SB_INLINE int sb_append_ulong(sb *sb, unsigned long v, int width, sb_pad_mode pad)
{
  int digits;

  if (pad == SB_PAD_NONE)
  {
    return sb_append_ulong_direct(sb, v);
  }

  digits = sb_count_digits_ulong(v);

  if (pad == SB_PAD_LEFT)
  {
    sb_append_spaces(sb, width - digits);
  }

  sb_append_ulong_direct(sb, v);

  if (pad == SB_PAD_RIGHT)
  {
    sb_append_spaces(sb, width - digits);
  }

  return digits;
}

SB_API SB_INLINE int sb_append_long(sb *sb, long v, int width, sb_pad_mode pad)
{
  int neg = 0;
  int digits;
  unsigned long u;

  if (v < 0L)
  {
    neg = 1;
    u = (unsigned long)(-(v + 1L)) + 1ul;
  }
  else
  {
    u = (unsigned long)v;
  }

  if (pad == SB_PAD_NONE)
  {
    if (neg)
    {
      sb_putc(sb, '-');
    }
    return (neg ? 1 : 0) + sb_append_ulong_direct(sb, u);
  }

  digits = sb_count_digits_ulong(u) + (neg ? 1 : 0);

  if (pad == SB_PAD_LEFT)
  {
    sb_append_spaces(sb, width - digits);
  }

  if (neg)
  {
    sb_putc(sb, '-');
  }

  sb_append_ulong_direct(sb, u);

  if (pad == SB_PAD_RIGHT)
  {
    sb_append_spaces(sb, width - digits);
  }

  return digits;
}

SB_API SB_INLINE int sb_append_double(sb *sb, double x, int width, int precision, sb_pad_mode pad)
{
  int neg = 0;
  unsigned long p10;
  unsigned long scaled;
  unsigned long ip;
  unsigned long frac_u;
  int ip_digits;
  int total_len;

  if (precision < 0)
  {
    precision = 0;
  }

  if (precision > 9)
  {
    precision = 9;
  }

  p10 = sb_pow10u(precision);

  if (x < 0.0)
  {
    neg = 1;
    x = -x;
  }

  scaled = (unsigned long)(x * (double)p10 + 0.5);
  ip = scaled / p10;
  frac_u = scaled - ip * p10;

  ip_digits = sb_count_digits_ulong(ip);
  total_len = ip_digits + (neg ? 1 : 0) + (precision > 0 ? (1 + precision) : 0);

  if (pad == SB_PAD_NONE)
  {
    if (neg)
    {
      sb_putc(sb, '-');
    }

    sb_append_ulong_direct(sb, ip);

    if (precision > 0)
    {
      int fdig;
      int leading;

      sb_putc(sb, '.');

      fdig = sb_count_digits_ulong(frac_u);
      leading = precision - fdig;

      while (leading-- > 0)
      {
        sb_putc(sb, '0');
      }

      sb_append_ulong_direct(sb, frac_u);
    }

    return total_len;
  }

  if (pad == SB_PAD_LEFT)
  {
    sb_append_spaces(sb, width - total_len);
  }

  if (neg)
  {
    sb_putc(sb, '-');
  }

  sb_append_ulong_direct(sb, ip);

  if (precision > 0)
  {
    int fdig;
    int leading;

    sb_putc(sb, '.');

    fdig = sb_count_digits_ulong(frac_u);
    leading = precision - fdig;

    while (leading-- > 0)
    {
      sb_putc(sb, '0');
    }

    sb_append_ulong_direct(sb, frac_u);
  }

  if (pad == SB_PAD_RIGHT)
  {
    sb_append_spaces(sb, width - total_len);
  }

  return total_len;
}

SB_API SB_INLINE int sb_append_float(sb *sb, float x, int width, int precision, sb_pad_mode pad)
{
  return sb_append_double(sb, (double)x, width, precision, pad);
}

SB_API SB_INLINE int sb_cmp(sb *sb, char *s)
{
  int i;

  for (i = 0; i < sb->len && s[i] != '\0'; ++i)
  {
    unsigned char c1 = (unsigned char)sb->buf[i];
    unsigned char c2 = (unsigned char)s[i];

    if (c1 != c2)
    {
      return c1 - c2;
    }
  }

  if (i < sb->len)
  {
    return (unsigned char)sb->buf[i];
  }

  if (s[i] != '\0')
  {
    return -(unsigned char)s[i];
  }

  return 0;
}

SB_API SB_INLINE int sb_ncmp(sb *sb, char *s, int n)
{
  int i;

  for (i = 0; i < n && i < sb->len && s[i] != '\0'; ++i)
  {
    unsigned char c1 = (unsigned char)sb->buf[i];
    unsigned char c2 = (unsigned char)s[i];

    if (c1 != c2)
    {
      return c1 - c2;
    }
  }

  if (i == n)
  {
    return 0;
  }

  if (i < sb->len)
  {
    return (unsigned char)sb->buf[i];
  }

  if (s[i] != '\0')
  {
    return -(unsigned char)s[i];
  }

  return 0;
}

#endif /* SB_H */

/*
   ------------------------------------------------------------------------------
   This software is available under 2 licenses -- choose whichever you prefer.
   ------------------------------------------------------------------------------
   ALTERNATIVE A - MIT License
   Copyright (c) 2025 nickscha
   Permission is hereby granted, free of charge, to any person obtaining a copy of
   this software and associated documentation files (the "Software"), to deal in
   the Software without restriction, including without limitation the rights to
   use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies
   of the Software, and to permit persons to whom the Software is furnished to do
   so, subject to the following conditions:
   The above copyright notice and this permission notice shall be included in all
   copies or substantial portions of the Software.
   THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
   IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
   FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
   AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
   LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
   OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
   SOFTWARE.
   ------------------------------------------------------------------------------
   ALTERNATIVE B - Public Domain (www.unlicense.org)
   This is free and unencumbered software released into the public domain.
   Anyone is free to copy, modify, publish, use, compile, sell, or distribute this
   software, either in source code form or as a compiled binary, for any purpose,
   commercial or non-commercial, and by any means.
   In jurisdictions that recognize copyright laws, the author or authors of this
   software dedicate any and all copyright interest in the software to the public
   domain. We make this dedication for the benefit of the public at large and to
   the detriment of our heirs and successors. We intend this dedication to be an
   overt act of relinquishment in perpetuity of all present and future rights to
   this software under copyright law.
   THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
   IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
   FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
   AUTHORS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN
   ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION
   WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
   ------------------------------------------------------------------------------
*/
