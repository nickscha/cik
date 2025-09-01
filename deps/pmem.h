/* pmem.h - v0.2 - public domain data structures - nickscha 2025

A C89 standard compliant, single header, nostdlib (no C Standard Library) Platform Memory Provider (PMEM).

LICENSE

  Placed in the public domain and also MIT licensed.
  See end of file for detailed license information.

*/
#ifndef PMEM_H
#define PMEM_H

/* #############################################################################
 * # COMPILER SETTINGS
 * #############################################################################
 */
/* Check if using C99 or later (inline is supported) */
#if __STDC_VERSION__ >= 199901L
#define PMEM_INLINE inline
#define PMEM_API static
#elif defined(__GNUC__) || defined(__clang__)
#define PMEM_INLINE __inline__
#define PMEM_API static
#elif defined(_MSC_VER)
#define PMEM_INLINE __inline
#define PMEM_API static
#else
#define PMEM_INLINE
#define PMEM_API static
#endif

typedef struct pmem
{
  unsigned long memory_size;
  void *memory;

} pmem;

/* #############################################################################
 * # WIN32 Implementation
 * #############################################################################
 */
#ifdef _WIN32

/* Because windows.h is massivly bloated and increases compilation time a lot we declare only the function we really need */
#ifndef _WINDOWS_
#define PMEM_WIN32_API(r) __declspec(dllimport) r __stdcall

#define MEM_COMMIT 0x00001000
#define MEM_RELEASE 0x8000
#define MEM_RESERVE 0x00002000
#define PAGE_READWRITE 0x04

PMEM_WIN32_API(void *)
VirtualAlloc(void *lpAddress, unsigned long dwSize, unsigned longflAllocationType, unsigned longflProtect);

PMEM_WIN32_API(int)
VirtualFree(void *lpAddress, unsigned long dwSize, unsigned long dwFreeType);

#endif /* _WINDOWS_ */

PMEM_API PMEM_INLINE int pmem_allocate(pmem *memory_struct)
{
  if (memory_struct == (void *)0 || memory_struct->memory_size == 0)
  {
    return 0;
  }

  /* Check if already allocated */
  if (memory_struct->memory != (void *)0)
  {
    return 0;
  }

  memory_struct->memory = (void *)VirtualAlloc(0, memory_struct->memory_size, MEM_COMMIT | MEM_RESERVE, PAGE_READWRITE);

  if (memory_struct->memory == (void *)0)
  {
    return 0;
  }

  return 1;
}

PMEM_API PMEM_INLINE int pmem_free(pmem *memory_struct)
{
  if (memory_struct == (void *)0 || memory_struct->memory == (void *)0)
  {
    return 0;
  }

  if (VirtualFree(memory_struct->memory, 0, MEM_RELEASE))
  {
    memory_struct->memory = (void *)0;
    return 1;
  }

  return 0;
}
#endif /* _WINDOWS_ */

/* #############################################################################
 * # LINUX Implementation
 * #############################################################################
 */
#ifdef __linux__

#ifndef _SYS_MMAN_H
#define PROT_READ 0x1      /* Page can be read.  */
#define PROT_WRITE 0x2     /* Page can be written.  */
#define MAP_PRIVATE 0x02   /* Changes are private.  */
#define MAP_ANONYMOUS 0x20 /* Don't use a file.  */
#define MAP_FAILED ((void *)-1)

void *mmap(void *addr, unsigned long len, int prot, int flags, int fd, long off);
int munmap(void *addr, unsigned long len);

#endif /* _SYS_MMAN_H */

PMEM_API PMEM_INLINE int pmem_allocate(pmem *memory_struct)
{
  if (memory_struct == (void *)0 || memory_struct->memory_size == 0)
  {
    return 0;
  }

  /* Check if already allocated */
  if (memory_struct->memory != (void *)0)
  {
    return 0;
  }

  memory_struct->memory = mmap(0, memory_struct->memory_size, PROT_READ | PROT_WRITE, MAP_PRIVATE | MAP_ANONYMOUS, -1, 0);

  if (memory_struct->memory == MAP_FAILED)
  {
    memory_struct->memory = (void *)0;
    return 0;
  }

  return 1;
}

PMEM_API PMEM_INLINE int pmem_free(pmem *memory_struct)
{
  if (memory_struct == (void *)0 || memory_struct->memory == (void *)0)
  {
    return 0;
  }

  if (munmap(memory_struct->memory, memory_struct->memory_size) == 0)
  {
    memory_struct->memory = (void *)0;
    return 1;
  }

  return 0;
}

#endif /* __linux__ */

/* #############################################################################
 * # MACOS Implementation
 * #############################################################################
 */
#ifdef __APPLE__

/* Manual declarations for mmap and munmap from sys/mman.h */
#ifndef _SYS_MMAN_H
#define PROT_READ 0x1
#define PROT_WRITE 0x2
#define MAP_PRIVATE 0x02
#define MAP_ANON 0x1000
#define MAP_FAILED ((void *)-1)

void *mmap(void *addr, unsigned long len, int prot, int flags, int fd, long off);
int munmap(void *addr, unsigned long len);

#endif /* _SYS_MMAN_H */

PMEM_API PMEM_INLINE int pmem_allocate(pmem *memory_struct)
{
  if (memory_struct == (void *)0 || memory_struct->memory_size == 0)
  {
    return 0;
  }

  /* Check if already allocated */
  if (memory_struct->memory != (void *)0)
  {
    return 0;
  }

  memory_struct->memory = mmap(0, memory_struct->memory_size, PROT_READ | PROT_WRITE, MAP_PRIVATE | MAP_ANON, -1, 0);

  if (memory_struct->memory == MAP_FAILED)
  {
    memory_struct->memory = (void *)0;
    return 0;
  }

  return 1;
}

PMEM_API PMEM_INLINE int pmem_free(pmem *memory_struct)
{
  if (memory_struct == (void *)0 || memory_struct->memory == (void *)0)
  {
    return 0;
  }

  if (munmap(memory_struct->memory, memory_struct->memory_size) == 0)
  {
    memory_struct->memory = (void *)0;
    return 1;
  }

  return 0;
}

#endif /* __APPLE__ */

#endif /* PMEM_H */

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
