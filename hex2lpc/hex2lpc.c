/*

	hex2lpc.c

	

*/

#include <stdio.h>
#include <stdarg.h>
#include <stdlib.h>
#include <string.h>

#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>

#include <time.h>

/* forward declarations */
int fmem_store_data(unsigned long adr, unsigned long cnt, unsigned char *data);


/*================================================*/
/* error procedure */
void err(char *fmt, ...)
{
	va_list va;
	va_start(va, fmt);
	vprintf(fmt, va);
	printf("\n");
	va_end(va);
}

/* user msg */
void msg(char *fmt, ...)
{
	va_list va;
	va_start(va, fmt);
	vprintf(fmt, va);
	printf("\n");
	va_end(va);
}


/*================================================*/
/* flash memory management */

/* mb = memory block */
struct _mb_struct
{
	unsigned long adr;	/* adr for "data" inside the target controller */
	unsigned long cnt;	/* number of bytes in "data" */
	unsigned char *data;	/* pointer to some data */
};
typedef struct _mb_struct mb_struct;

mb_struct *mb_open(void)
{
	mb_struct *mb = (mb_struct *)malloc(sizeof(mb_struct));
	if ( mb != NULL )
	{
		mb->adr = 0UL;
		mb->cnt = 0UL;
		mb->data = NULL;
		return mb;
	}
	return err("fmem: mem block init error"), NULL;
}

void mb_close(mb_struct *mb)
{
	if ( mb == NULL )
		return;
	mb->cnt = 0UL;
	if ( mb->data != NULL )
		free(mb->data);
	free(mb->data);
}

/* set size of the memory block, existing data will be preserved, returns 0 for error */
int mb_set_data_size(mb_struct *mb, unsigned long cnt)
{
	if ( mb->data == NULL )
	{
		mb->data = (unsigned char *)malloc(cnt);
		if ( mb->data == NULL )
			return err("fmem: mem data alloc error"), 0;
	}
	else
	{
		void *ptr;
		ptr = realloc(mb->data , cnt);
		if ( ptr == NULL )
			return err("fmem: mem data realloc error"), 0;
		mb->data = (unsigned char *)ptr;
	}
	
	mb->cnt = cnt;
	return 1;
}

mb_struct **fmem_mb_list = NULL;
unsigned long fmem_mb_list_cnt = 0UL;
unsigned long fmem_mb_list_max = 0UL;
#define FMEM_EXPAND 32

int fmem_init(void)
{
	fmem_mb_list = (mb_struct **)malloc(sizeof(mb_struct *)*FMEM_EXPAND);
	if ( fmem_mb_list == NULL )
		return err("fmem: mem block init error"), 0;
	fmem_mb_list_cnt = 0UL;
	fmem_mb_list_max = 0UL;
	return 1;
}

int fmem_expand(void)
{
	void *ptr;
	ptr = realloc(fmem_mb_list, (fmem_mb_list_max+FMEM_EXPAND)*sizeof(mb_struct *));
	if ( ptr == NULL )
		return err("fmem: mem block list expand error"), 0;
	fmem_mb_list = (mb_struct **)ptr;
	fmem_mb_list_max+= FMEM_EXPAND;
	return 1;
}

int fmem_add_data(unsigned long adr, unsigned long cnt, unsigned char *data)
{
	mb_struct *mb = mb_open();
	if ( mb != NULL )
	{
		if ( mb_set_data_size(mb, cnt) != 0 )
		{
			while( fmem_mb_list_cnt >= fmem_mb_list_max )
				if ( fmem_expand() == 0 )
				{
					mb_close(mb);
					return 0;
				}
				
			fmem_mb_list[fmem_mb_list_cnt] = mb;
			fmem_mb_list_cnt++;
			return 1;
		}
	}
	return 0;
}

int fmem_store_data(unsigned long adr, unsigned long cnt, unsigned char *data)
{
	if ( fmem_mb_list_cnt > 0 )
	{
		mb_struct *mb = fmem_mb_list[fmem_mb_list_cnt-1];
		if ( mb->adr + mb->cnt == adr )
		{
			unsigned long old_cnt = mb->cnt;
			if ( mb_set_data_size(mb, mb->cnt + cnt) == 0 )
				return 0;
			memcpy(mb->data + old_cnt, data, cnt);
			return 1;
		}
	}
	return fmem_add_data(adr, cnt, data);
}

void fmem_show(void)
{
	unsigned long i;
	mb_struct *mb;
	for( i = 0; i < fmem_mb_list_cnt; i++ )
	{
		mb = fmem_mb_list[i];
		printf("%lu/%lu: adr=0x%08lx cnt=%lu\n", i+1, fmem_mb_list_cnt, mb->adr, mb->cnt);
	}
}

/*================================================*/
/* intel hex parser */

#define IHEX_LINE_LEN 1024
#define IHEX_DATA_LEN 300
FILE *ihex_fp;
char ihex_line[IHEX_LINE_LEN];
int ihex_pos = 0;
int ihex_curr_line = 0;

unsigned long ihex_line_bytes;
unsigned long ihex_line_extsegadr = 0UL;	/* from typ 02 */
unsigned long ihex_line_extlinadr = 0UL;		/* from typ 04 */
unsigned long ihex_line_adr;
unsigned long ihex_line_type;
unsigned char ihex_line_data[IHEX_DATA_LEN];

/* calculate the current address */
unsigned long ihex_get_curr_adr(void)
{
	unsigned long adr;
	adr = 0UL;
	adr += ihex_line_extlinadr<<16;
	adr += ihex_line_extsegadr<<16;
	adr += ihex_line_adr;
	return adr;
}

/* read one line from the stream (global variable), return 0 for error or EOF */
int ihex_read_line(void)
{
	if ( ihex_fp == NULL )
		return err("ihex line %lu: internal error (file ptr)", ihex_curr_line), 0;
	if ( fgets(ihex_line, IHEX_LINE_LEN, ihex_fp) == NULL )
		return 0;
	ihex_line[IHEX_LINE_LEN-1] = '\0';
	ihex_pos = 0;
	ihex_curr_line++;
	return 1;
}

/* get a char from the internal line buffer and goto to next char */
unsigned int ihex_getc(void)
{
	unsigned int c;
	c = ihex_line[ihex_pos];
	if ( c != '\0' )
		ihex_pos++;
	return c;
}

unsigned int ihex_getchex(void)
{
	unsigned int c = ihex_getc();
	if ( c < '0' )
		return 0;
	if ( c <= '9' )
		return c - '0';
	if ( c < 'A' )
		return 0;
	if ( c <= 'F' )
		return c - 'A' + 10U;
	if ( c < 'a' )
		return 0;
	if ( c <= 'f' )
		return c - 'a' + 10U;
	return 0;
		
}

unsigned long ihex_gethex(int size)
{
	unsigned long v = 0;
	
	while( size > 0 )
	{
		v *= 16;
		v += ihex_getchex();
		size--;
	}
	return v;
}

/* parse line in global buffer ihex_line, return 0 on error, return 1 for eof, otherwise 2 */
int ihex_parse_line(void)
{
	unsigned long i;
	unsigned long sum = 0;
	if ( ihex_getc() != ':' )
		return 0;
	
	ihex_line_bytes = ihex_gethex(2);
	sum += ihex_line_bytes;
	ihex_line_adr = ihex_gethex(4);
	sum += ihex_line_adr & 0x0ff;
	sum += (ihex_line_adr>>8) & 0x0ff;
	ihex_line_type = ihex_gethex(2);
	sum += ihex_line_type & 0x0ff;
	switch(ihex_line_type)
	{
		case 0: 	/* data record */
			
			for( i = 0; i <= ihex_line_bytes /* includes checksum */; i++ )
			{
				ihex_line_data[i] = ihex_gethex(2);
				sum += ihex_line_data[i];
				// printf("%02lx: %02x/%02lx\n", i, ihex_line_data[i], sum);
			}
			if ( (sum & 0x0ff) != 0 ) 
				return err("ihex line %lu: checksum error", ihex_curr_line), 0;		/* checksum error */
			/* call flash memory management */
			if ( fmem_store_data( ihex_get_curr_adr(), ihex_line_bytes, ihex_line_data) == 0 )
				return err("ihex line %lu: store memory error", ihex_curr_line), 0;
			break;
		case 1:	/* end of file */
			return 1;
		case 2:	/* extended segment adr */
			ihex_line_extsegadr = ihex_gethex(4);
			break;
		case 3:	/* start extended segment adr */
			/* ignored */
			break;
		case 4: /* linear adr */
			ihex_line_extlinadr = ihex_gethex(4);
			break;
		case 5:	/* start linear adr */
			/* ignored */
			break;
		default:
			return err("ihex line %lu: unknown type %lu", ihex_curr_line, ihex_line_type), 0;	/* unknown type */
	}
	return 2;
}

int ihex_read_fp(void)
{
	if ( fmem_init() == 0 )
		return 0;
	for(;;)
	{
		if ( ihex_read_line() == 0 )
			break;
		if ( ihex_parse_line() == 0 )
			return 0;
	}
	return 1;
}

int ihex_read_file(const char *filename)
{
	ihex_fp = fopen(filename, "rb");
	if ( ihex_fp == NULL )
		return err("ihex: open error %s", filename), 0;
	if ( ihex_read_fp() == 0 )
	{
		fclose(ihex_fp);
		return 0;
	}
	fclose(ihex_fp);
	return 1;
}

/*================================================*/
/* uart connection  */

int uart_fd = 0;
struct termios uart_io;
/* in_buf should be large enough to read a complete sector with some additional overhead */
#define UART_IN_BUF_LEN (1024*48)
unsigned char uart_in_buf[UART_IN_BUF_LEN];
unsigned long uart_in_pos = 0;


int uart_is_synchronized = 0;

void uart_add_in_buf(unsigned char c)
{
	if ( uart_in_pos < UART_IN_BUF_LEN )
		uart_in_buf[uart_in_pos++] = c;
}

void uart_reset_in_buf(void)
{
	uart_in_pos = 0;
}

void uart_show_in_buf(void)
{
  unsigned long i,j;
  i = 0;
  while( i < uart_in_pos )
  {
    for( j = 0; j < 16; j++ )
    {
      if ( i + j < uart_in_pos )
	printf("%02x ", uart_in_buf[i+j]);
      else
	printf("   ");
    }

    for( j = 0; j < 16; j++ )
    {
      if ( i + j < uart_in_pos && uart_in_buf[i+j] >= 32 && uart_in_buf[i+j] <= 127 )
	printf("%c", uart_in_buf[i+j]);
      else
	printf(".");
    }
    printf("\n");
    i += j;
  }
}

/* get the LPC error/result code from the end of the buffer */
int uart_get_result_code(void)
{
  int code = 0;
  unsigned long i = uart_in_pos;
  if ( i == 0 )
    return -1;
  i--;
  if ( uart_in_buf[i] == '\n' || uart_in_buf[i] == '\r' )
  {
    if ( i == 0 )
      return -1;
    i--;
  }
  if ( uart_in_buf[i] == '\n' || uart_in_buf[i] == '\r' )
  {
    if ( i == 0 )
      return -1;
    i--;
  }
  if ( uart_in_buf[i] >= '0' && uart_in_buf[i] <= '9' )
  {
    code = uart_in_buf[i] - '0';
    if ( i == 0 )
      return code;
    i--;
    if ( uart_in_buf[i] >= '0' && uart_in_buf[i] <= '9' )
    {
      code = (uart_in_buf[i] - '0')*10 + code;
    }
    return code;
  }
  return -1;
  
}

/*
	values for baud are:
		B1200
		B1800
		B2400
		B4800
		B9600
		B19200
		B38400
		B57600
		B115200
		B230400
*/
int uart_open(const char *device, int baud)
{
	uart_is_synchronized = 0;
	uart_reset_in_buf();
	
	uart_fd=open(device, O_RDWR|O_NONBLOCK|O_NDELAY);
	if ( uart_fd < 0 )
		return perror(device), err("uart: %s error", device), 0;
	if ( tcgetattr(uart_fd, &uart_io) < 0 )
		return close(uart_fd), err("uart: tcgetattr error"), 0;
	
	uart_io.c_iflag = IGNBRK | IGNPAR;
	uart_io.c_oflag = 0;
	uart_io.c_cflag = CS8|CREAD|CLOCAL;
	uart_io.c_lflag = 0;		/* none canonical mode */
	uart_io.c_cc[VMIN] = 0;	/* make chars available immeadiatly in none canonical mode */
	uart_io.c_cc[VTIME] = 0;
	
	if ( tcsetattr(uart_fd, TCSANOW, &uart_io) < 0 )
		return close(uart_fd), err("uart: tcsetattr error"), 0;
	
	
	if ( cfsetispeed(&uart_io, baud) < 0 )
		return close(uart_fd), err("uart: cfsetispeed error"), 0;
	if ( cfsetospeed(&uart_io, baud) < 0 )
		return close(uart_fd), err("uart: cfsetospeed error"), 0;

	return 1;	
}

int uart_read_byte(void)
{
	ssize_t cnt;
	if ( uart_fd < 0 )
		return -1;
	unsigned char buf[2];
	cnt = read(uart_fd, buf, 1);
	if ( cnt == 0 )
		return -1;
	uart_add_in_buf(buf[0]);
	return buf[0];
}

int uart_send_byte(int c)
{
	unsigned char buf[1];
	buf[0] = c;
	if ( uart_fd >= 0 )
	{
		write(uart_fd, buf, 1);
		uart_read_byte();
	}
	return 1;
}

int uart_send_str(const char *str)
{
	int i, len = strlen(str);
	for( i = 0; i < len; i++ )
		if ( uart_send_byte(str[i]) == 0 )
			return 0;
	return 1;
}

#ifdef USE_TIME
void uart_read_more(void)
{
	time_t start;
	time_t curr;
	int c;
	unsigned long received_chars;

	time(&start);
	received_chars = 0;
	for(;;)
	{
		time(&curr);
		if ( start + 2 <= curr )
			break;
		c =uart_read_byte();
		if ( c >= 0 )
		{
			// printf("[%lu %d %c]\n", received_chars, c, c < ' ' ? ' ' : c);
			time(&start);
			received_chars++;
		}
	}
	// printf("received_chars=%lu\n", received_chars);
}
#else
void uart_read_more(void)
{
	clock_t start;
	clock_t curr;
	int c;
	unsigned long received_chars;

	start = clock();
	received_chars = 0;
	for(;;)
	{
		curr = clock();
		if ( start + CLOCKS_PER_SEC/4 <= curr )
			break;
		c =uart_read_byte();
		if ( c >= 0 )
		{
			// printf("[%lu %d %c]\n", received_chars, c, c < ' ' ? ' ' : c);
			start = clock();			/* reset clock */
			received_chars++;
		}
	}
	// printf("received_chars=%lu\n", received_chars);
}
#endif 

/*================================================*/
/* LPC communication  */


/*
	set the uart_is_synchronized flag
	return the flag value.
*/
#define UART_STR_SYNCHRONIZED "Synchronized"
#define UART_STR_12000 "12000"

/* return 0 if not in sync */
int uart_send_startup(void)
{
	uart_reset_in_buf();
	uart_send_str(UART_STR_SYNCHRONIZED "\r\n");
	uart_read_more();
	if ( strncmp((const char *)uart_in_buf, UART_STR_SYNCHRONIZED, strlen(UART_STR_SYNCHRONIZED)) == 0 )
	{
		/* yes, synchronized, send clock speed */
		uart_reset_in_buf();
		uart_send_str(UART_STR_12000 "\r\n");
		uart_read_more();		
		if ( strncmp((const char *)uart_in_buf, UART_STR_12000, strlen(UART_STR_12000)) == 0 )
		{
			/* yes, synchronized */
			return 1;
		}
	}
	return 0;
}


int uart_synchronize(int is_retry_quiet)
{
	if ( is_retry_quiet == 0 )
		msg("send '?'");
	uart_reset_in_buf();
	uart_send_str("?");
	uart_read_more();
	
	if ( uart_in_pos == 0 )
	{
		if ( is_retry_quiet == 0 )
			msg("no response");
		uart_is_synchronized = 0;
		
	}
	else
	{
		/* echo received, this could mean, that the device is already syncronized */
		if ( uart_in_buf[0] == '?' )
		{
			if ( is_retry_quiet == 0 )
				msg("echo received, try startup sequence");
			/* the input was echoed, looks like it is synchronized */
			/* to ensure proper operation, the startup sequence is repeated */
			
			uart_is_synchronized = uart_send_startup();
		}
		else
		{
			/* check if the controller did response with the synchronized command */
			if ( strncmp((const char *)uart_in_buf, UART_STR_SYNCHRONIZED, strlen(UART_STR_SYNCHRONIZED)) == 0 )
			{
				/* "Synchronized" recevied, send startup */
				msg("controller synchronize request");
				uart_is_synchronized = uart_send_startup();
			}
			else
			{
				if ( is_retry_quiet == 0 )
					msg("incorrect response from controller");
				uart_is_synchronized = 0;				
			}			
		}
	}
	if ( uart_is_synchronized != 0 )
		msg("controller is in sync");
	if ( is_retry_quiet == 0 )
	{
		if ( uart_is_synchronized == 0 )
			msg("controller is NOT in sync");
	}
	return uart_is_synchronized;
}

/*
  Read from controller memory. Result will be placed in uart_in_buf
  Return value contains the start index in uart_in_buf or is -1 if the read has failed
*/
long uart_read_from_adr(unsigned long adr, unsigned long cnt)
{
  char s[32];
  if ( cnt > UART_IN_BUF_LEN-64 )
    return err("wrong args for read memory"), -1;
  sprintf(s, "R %lu %lu\r\n", adr, cnt);
  uart_reset_in_buf();
  uart_send_str(s);
  uart_read_more();
  if ( uart_in_pos < 3+cnt )
    return err("read memory failed (too less data)"), -1;
  
  /* check for success code */
  if ( uart_in_buf[uart_in_pos-cnt-3] != '0'  )
    return err("read memory failed (illegal return code)"), -1;
  
  //printf("read operation, uart_in_pos = %lu, result stats at %ld\n", uart_in_pos, uart_in_pos-cnt);
  return uart_in_pos-cnt;
}

/*
  read part number, also returned as result
*/
unsigned long uart_read_part_numer(void)
{
  unsigned long id, cnt, i;
  uart_reset_in_buf();
  uart_send_str("J\r\n");
  uart_read_more();
  if ( uart_in_pos < 5 )
    return err("read part number failed (too less data)"), 0;

  uart_in_buf[uart_in_pos] = '\0';
  
  /* check for success code */
  i = 0;
  while( i < uart_in_pos )
  {
    if ( uart_in_buf[i] == '0' )
    {
      i++;
      while( uart_in_buf[i] == '\r' || uart_in_buf[i] == '\n')
      {
	i++;
      }
      id = strtoul(uart_in_buf+i, NULL, 10);
      return id;
    }
    i++;
  }
  return err("read part number failed (illegal return code)"), 0;
  
}


/*================================================*/
/* LPC type identification */

struct _lpc_struct
{
  char *name;
  unsigned long part_id;
  unsigned long flash_size;
  unsigned long sector_size;
};
typedef struct _lpc_struct lpc_struct;

lpc_struct lpc_list[] = {
/* name, part_id, flash_size, sector_size */
{"LPC810M021FN8", 0x00008100, 0x1000,0x0400 },
{"LPC811M001JDH16", 0x00008110, 0x2000,0x0400 },
{"LPC812M101JDH16", 0x00008120, 0x4000,0x0400 },
{"LPC812M101JD20", 0x00008121, 0x4000,0x0400 },
{"LPC812M101JDH20", 0x00008122, 0x4000,0x0400 },
{"LPC812M101JTB16", 0x00008122, 0x4000,0x0400 }
};

unsigned long lpc_part_id = 0;
lpc_struct *lpc_part = NULL;

lpc_struct *lpc_find_by_part_id(unsigned long part_id)
{
  int i;
  for( i = 0; i < sizeof(lpc_list)/sizeof(*lpc_list); i++ )
  {
    if ( part_id == lpc_list[i].part_id )
      return lpc_list+i;
  }
  return NULL;
}

/*
  requires, that lpc_part has been set correctly
*/
int lpc_erase_all(void)
{
  int i;
  char s[32];
  unsigned long sector_cnt;
  int result_code;

  if ( lpc_part == NULL )
    return 0;

  sector_cnt = lpc_part->flash_size / lpc_part->sector_size;
  sector_cnt--;
  sprintf(s, "E 0 %lu\r\n", sector_cnt);
  uart_reset_in_buf();
  uart_send_str(s);

  for( i = 0; i < 4*5; i++)
  {
    uart_read_more();
    
    result_code = uart_get_result_code();
    if ( result_code > 0 )
      return err("flash erase failed (%d)", result_code), 0;
    else if ( result_code == 0 )
      return 1;
  }
  return err("flash timeout"), 0;
  
  //uart_show_in_buf();
  
  return 1;
  
}

/*================================================*/
/* main */

int main(int argc, char **argv)
{
	ihex_read_file("lpc810_led_driver_700mA.hex");
	fmem_show();
	uart_open("/dev/ttyUSB0", B9600);
	if ( uart_synchronize(0) == 0 )
	{
		for(;;)
		{
			msg("please put controller into UART ISP mode, retry...");
			if ( uart_synchronize(1) != 0 )
				break;
		}
	}
	
	/* read part number */
	lpc_part_id = uart_read_part_numer();
	msg("received part id 0x%08x", lpc_part_id);
	
	/*  check if the part number is known */
	lpc_part = lpc_find_by_part_id(lpc_part_id);
	if ( lpc_part == NULL )
	{
	  err("unknown controller");
	  return 0;
	}
	
	msg("controller %s with %lu bytes flash memory", lpc_part->name, lpc_part->flash_size);
	
	lpc_erase_all();
	
	//uart_read_from_adr(0, 32);
	uart_show_in_buf();
	
	return 0;
}




