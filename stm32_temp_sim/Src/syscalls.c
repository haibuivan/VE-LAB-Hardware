/****************************
 * file: syscall.c
 * author: haihbv
 * note: syscall cho USART và FreeRTOS
 *       _write: để printf có thể gửi dữ liệu qua USART
 *       _sbrk: để malloc/free có thể hoạt động
 ********************/

#include <sys/stat.h>
#include <sys/types.h>
#include <errno.h>
#include <stdio.h>
#include <string.h>

#include "stm32f10x_usart.h"

// volatile unsigned long g_write_call_count = 0; // debug xem có bao nhiêu lần _write được gọi

int _write(int file, char *ptr, int len)
{
    int i;

    // g_write_call_count++;

    if ((file != 1) && (file != 2))
    {
        errno = EBADF;
        return -1;
    }

    // thay đổi USART debug ở đây hiện tại tôi đang dùng USART1
    for (i = 0; i < len; i++)
    {
        if (ptr[i] == '\n')
        {
            while (USART_GetFlagStatus(USART1, USART_FLAG_TXE) == RESET);
            USART_SendData(USART1, '\r');
        }

        while (USART_GetFlagStatus(USART1, USART_FLAG_TXE) == RESET);
        USART_SendData(USART1, ptr[i]);
    }

    return len;
}

/* ================= MEMORY ================= */

/* linker sẽ cung cấp symbol này */
extern char _end;
static char *heap_end;

caddr_t _sbrk(int incr)
{
    extern char _estack; // cuối RAM
    char *prev_heap_end;

    if (heap_end == 0)
    {
        heap_end = &_end;
    }

    prev_heap_end = heap_end;

    if (heap_end + incr > &_estack)
    {
        errno = ENOMEM;
        return (caddr_t)-1;
    }

    heap_end += incr;
    return (caddr_t)prev_heap_end;
}

/* ================= STUB FUNCTIONS ================= */

int _close(int file)
{
    return -1;
}

int _fstat(int file, struct stat *st)
{
    st->st_mode = S_IFCHR;
    return 0;
}

int _isatty(int file)
{
    return 1;
}

int _lseek(int file, int ptr, int dir)
{
    return 0;
}

int _read(int file, char *ptr, int len)
{
    errno = EINVAL;
    return -1;
}

int _open(const char *name, int flags, int mode)
{
    return -1;
}

int _kill(int pid, int sig)
{
    errno = EINVAL;
    return -1;
}

int _getpid(void)
{
    return 1;
}
