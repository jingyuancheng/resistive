#define nx_struct struct
#define nx_union union
#define dbg(mode, format, ...) ((void)0)
#define dbg_clear(mode, format, ...) ((void)0)
#define dbg_active(mode) 0
# 151 "/opt/local/lib/gcc-lib/msp430/3.2.3/include/stddef.h" 3
typedef int ptrdiff_t;
#line 213
typedef unsigned int size_t;
#line 325
typedef int wchar_t;
# 8 "/opt/local/lib/ncc/deputy_nodeputy.h"
struct __nesc_attr_nonnull {
#line 8
  int dummy;
}  ;
#line 9
struct __nesc_attr_bnd {
#line 9
  void *lo, *hi;
}  ;
#line 10
struct __nesc_attr_bnd_nok {
#line 10
  void *lo, *hi;
}  ;
#line 11
struct __nesc_attr_count {
#line 11
  int n;
}  ;
#line 12
struct __nesc_attr_count_nok {
#line 12
  int n;
}  ;
#line 13
struct __nesc_attr_one {
#line 13
  int dummy;
}  ;
#line 14
struct __nesc_attr_one_nok {
#line 14
  int dummy;
}  ;
#line 15
struct __nesc_attr_dmemset {
#line 15
  int a1, a2, a3;
}  ;
#line 16
struct __nesc_attr_dmemcpy {
#line 16
  int a1, a2, a3;
}  ;
#line 17
struct __nesc_attr_nts {
#line 17
  int dummy;
}  ;
# 38 "/opt/local/msp430/include/sys/inttypes.h" 3
typedef signed char int8_t;
typedef unsigned char uint8_t;

typedef int int16_t;
typedef unsigned int uint16_t;

typedef long int32_t;
typedef unsigned long uint32_t;

typedef long long int64_t;
typedef unsigned long long uint64_t;




typedef int16_t intptr_t;
typedef uint16_t uintptr_t;
# 431 "/opt/local/lib/ncc/nesc_nx.h"
typedef struct { unsigned char nxdata[1]; } __attribute__((packed)) nx_int8_t;typedef int8_t __nesc_nxbase_nx_int8_t  ;
typedef struct { unsigned char nxdata[2]; } __attribute__((packed)) nx_int16_t;typedef int16_t __nesc_nxbase_nx_int16_t  ;
typedef struct { unsigned char nxdata[4]; } __attribute__((packed)) nx_int32_t;typedef int32_t __nesc_nxbase_nx_int32_t  ;
typedef struct { unsigned char nxdata[8]; } __attribute__((packed)) nx_int64_t;typedef int64_t __nesc_nxbase_nx_int64_t  ;
typedef struct { unsigned char nxdata[1]; } __attribute__((packed)) nx_uint8_t;typedef uint8_t __nesc_nxbase_nx_uint8_t  ;
typedef struct { unsigned char nxdata[2]; } __attribute__((packed)) nx_uint16_t;typedef uint16_t __nesc_nxbase_nx_uint16_t  ;
typedef struct { unsigned char nxdata[4]; } __attribute__((packed)) nx_uint32_t;typedef uint32_t __nesc_nxbase_nx_uint32_t  ;
typedef struct { unsigned char nxdata[8]; } __attribute__((packed)) nx_uint64_t;typedef uint64_t __nesc_nxbase_nx_uint64_t  ;


typedef struct { unsigned char nxdata[1]; } __attribute__((packed)) nxle_int8_t;typedef int8_t __nesc_nxbase_nxle_int8_t  ;
typedef struct { unsigned char nxdata[2]; } __attribute__((packed)) nxle_int16_t;typedef int16_t __nesc_nxbase_nxle_int16_t  ;
typedef struct { unsigned char nxdata[4]; } __attribute__((packed)) nxle_int32_t;typedef int32_t __nesc_nxbase_nxle_int32_t  ;
typedef struct { unsigned char nxdata[8]; } __attribute__((packed)) nxle_int64_t;typedef int64_t __nesc_nxbase_nxle_int64_t  ;
typedef struct { unsigned char nxdata[1]; } __attribute__((packed)) nxle_uint8_t;typedef uint8_t __nesc_nxbase_nxle_uint8_t  ;
typedef struct { unsigned char nxdata[2]; } __attribute__((packed)) nxle_uint16_t;typedef uint16_t __nesc_nxbase_nxle_uint16_t  ;
typedef struct { unsigned char nxdata[4]; } __attribute__((packed)) nxle_uint32_t;typedef uint32_t __nesc_nxbase_nxle_uint32_t  ;
typedef struct { unsigned char nxdata[8]; } __attribute__((packed)) nxle_uint64_t;typedef uint64_t __nesc_nxbase_nxle_uint64_t  ;
# 41 "/opt/local/msp430/include/sys/types.h" 3
typedef unsigned char u_char;
typedef unsigned short u_short;
typedef unsigned int u_int;
typedef unsigned long u_long;
typedef unsigned short ushort;
typedef unsigned int uint;

typedef uint8_t u_int8_t;
typedef uint16_t u_int16_t;
typedef uint32_t u_int32_t;
typedef uint64_t u_int64_t;

typedef u_int64_t u_quad_t;
typedef int64_t quad_t;
typedef quad_t *qaddr_t;

typedef char *caddr_t;
typedef const char *c_caddr_t;
typedef volatile char *v_caddr_t;
typedef u_int32_t fixpt_t;
typedef u_int32_t gid_t;
typedef u_int32_t in_addr_t;
typedef u_int16_t in_port_t;
typedef u_int32_t ino_t;
typedef long key_t;
typedef u_int16_t mode_t;
typedef u_int16_t nlink_t;
typedef quad_t rlim_t;
typedef int32_t segsz_t;
typedef int32_t swblk_t;
typedef int32_t ufs_daddr_t;
typedef int32_t ufs_time_t;
typedef u_int32_t uid_t;
# 59 "/opt/local/msp430/include/stdlib.h" 3
#line 56
typedef struct __nesc_unnamed4242 {
  int quot;
  int rem;
} div_t;







#line 64
typedef struct __nesc_unnamed4243 {
  long quot;
  long rem;
} ldiv_t;
# 122 "/opt/local/msp430/include/sys/config.h" 3
typedef long int __int32_t;
typedef unsigned long int __uint32_t;
# 12 "/opt/local/msp430/include/sys/_types.h" 3
typedef long _off_t;
typedef long _ssize_t;
# 28 "/opt/local/msp430/include/sys/reent.h" 3
typedef __uint32_t __ULong;


struct _glue {

  struct _glue *_next;
  int _niobs;
  struct __sFILE *_iobs;
};

struct _Bigint {

  struct _Bigint *_next;
  int _k, _maxwds, _sign, _wds;
  __ULong _x[1];
};


struct __tm {

  int __tm_sec;
  int __tm_min;
  int __tm_hour;
  int __tm_mday;
  int __tm_mon;
  int __tm_year;
  int __tm_wday;
  int __tm_yday;
  int __tm_isdst;
};







struct _atexit {
  struct _atexit *_next;
  int _ind;
  void (*_fns[32])(void );
};








struct __sbuf {
  unsigned char *_base;
  int _size;
};






typedef long _fpos_t;
#line 116
struct __sFILE {
  unsigned char *_p;
  int _r;
  int _w;
  short _flags;
  short _file;
  struct __sbuf _bf;
  int _lbfsize;


  void *_cookie;

  int (*_read)(void *_cookie, char *_buf, int _n);
  int (*_write)(void *_cookie, const char *_buf, int _n);

  _fpos_t (*_seek)(void *_cookie, _fpos_t _offset, int _whence);
  int (*_close)(void *_cookie);


  struct __sbuf _ub;
  unsigned char *_up;
  int _ur;


  unsigned char _ubuf[3];
  unsigned char _nbuf[1];


  struct __sbuf _lb;


  int _blksize;
  int _offset;

  struct _reent *_data;
};
#line 174
struct _rand48 {
  unsigned short _seed[3];
  unsigned short _mult[3];
  unsigned short _add;
};









struct _reent {


  int _errno;




  struct __sFILE *_stdin, *_stdout, *_stderr;

  int _inc;
  char _emergency[25];

  int _current_category;
  const char *_current_locale;

  int __sdidinit;

  void (*__cleanup)(struct _reent *arg_0x10079f880);


  struct _Bigint *_result;
  int _result_k;
  struct _Bigint *_p5s;
  struct _Bigint **_freelist;


  int _cvtlen;
  char *_cvtbuf;

  union __nesc_unnamed4244 {

    struct __nesc_unnamed4245 {

      unsigned int _unused_rand;
      char *_strtok_last;
      char _asctime_buf[26];
      struct __tm _localtime_buf;
      int _gamma_signgam;
      __extension__ unsigned long long _rand_next;
      struct _rand48 _r48;
    } _reent;



    struct __nesc_unnamed4246 {


      unsigned char *_nextf[30];
      unsigned int _nmalloc[30];
    } _unused;
  } _new;


  struct _atexit *_atexit;
  struct _atexit _atexit0;


  void (**_sig_func)(int arg_0x1007a28f8);




  struct _glue __sglue;
  struct __sFILE __sf[3];
};
#line 273
struct _reent;
# 18 "/opt/local/msp430/include/math.h" 3
union __dmath {

  __uint32_t i[2];
  double d;
};




union __dmath;
#line 208
struct exception {


  int type;
  char *name;
  double arg1;
  double arg2;
  double retval;
  int err;
};
#line 261
enum __fdlibm_version {

  __fdlibm_ieee = -1, 
  __fdlibm_svid, 
  __fdlibm_xopen, 
  __fdlibm_posix
};




enum __fdlibm_version;
# 91 "/Users/jingyuancheng/tinyos/moteiv/tos/system/tos.h"
typedef unsigned char bool;






enum __nesc_unnamed4247 {
  FALSE = 0, 
  TRUE = 1
};

uint16_t TOS_LOCAL_ADDRESS = 1;

enum __nesc_unnamed4248 {
  FAIL = 0, 
  SUCCESS = 1
};


struct __nesc_attr_atmostonce {
};
#line 112
struct __nesc_attr_atleastonce {
};
#line 113
struct __nesc_attr_exactlyonce {
};

static inline uint8_t rcombine(uint8_t r1, uint8_t r2);
typedef uint8_t result_t  ;







static inline result_t rcombine(result_t r1, result_t r2);
#line 145
enum __nesc_unnamed4249 {
  NULL = 0x0
};
# 39 "/opt/local/msp430/include/msp430/iostructures.h" 3
#line 27
typedef union port {
  volatile unsigned char reg_p;
  volatile struct __nesc_unnamed4250 {
    unsigned char __p0 : 1, 
    __p1 : 1, 
    __p2 : 1, 
    __p3 : 1, 
    __p4 : 1, 
    __p5 : 1, 
    __p6 : 1, 
    __p7 : 1;
  } __pin;
} __attribute((packed))  ioregister_t;
#line 108
struct port_full_t {
  ioregister_t in;
  ioregister_t out;
  ioregister_t dir;
  ioregister_t ifg;
  ioregister_t ies;
  ioregister_t ie;
  ioregister_t sel;
};









struct port_simple_t {
  ioregister_t in;
  ioregister_t out;
  ioregister_t dir;
  ioregister_t sel;
};




struct port_full_t;



struct port_full_t;



struct port_simple_t;



struct port_simple_t;



struct port_simple_t;



struct port_simple_t;
# 116 "/opt/local/msp430/include/msp430/gpio.h" 3
volatile unsigned char P1OUT __asm ("0x0021");

volatile unsigned char P1DIR __asm ("0x0022");



volatile unsigned char P1IES __asm ("0x0024");

volatile unsigned char P1IE __asm ("0x0025");

volatile unsigned char P1SEL __asm ("0x0026");










volatile unsigned char P2OUT __asm ("0x0029");

volatile unsigned char P2DIR __asm ("0x002A");





volatile unsigned char P2IE __asm ("0x002D");

volatile unsigned char P2SEL __asm ("0x002E");










volatile unsigned char P3OUT __asm ("0x0019");

volatile unsigned char P3DIR __asm ("0x001A");

volatile unsigned char P3SEL __asm ("0x001B");










volatile unsigned char P4OUT __asm ("0x001D");

volatile unsigned char P4DIR __asm ("0x001E");

volatile unsigned char P4SEL __asm ("0x001F");










volatile unsigned char P5OUT __asm ("0x0031");

volatile unsigned char P5DIR __asm ("0x0032");

volatile unsigned char P5SEL __asm ("0x0033");










volatile unsigned char P6OUT __asm ("0x0035");

volatile unsigned char P6DIR __asm ("0x0036");

volatile unsigned char P6SEL __asm ("0x0037");
# 92 "/opt/local/msp430/include/msp430/usart.h" 3
volatile unsigned char U0CTL __asm ("0x0070");

volatile unsigned char U0TCTL __asm ("0x0071");



volatile unsigned char U0MCTL __asm ("0x0073");

volatile unsigned char U0BR0 __asm ("0x0074");

volatile unsigned char U0BR1 __asm ("0x0075");

volatile unsigned char U0RXBUF __asm ("0x0076");
#line 275
volatile unsigned char U1CTL __asm ("0x0078");

volatile unsigned char U1TCTL __asm ("0x0079");

volatile unsigned char U1RCTL __asm ("0x007A");

volatile unsigned char U1MCTL __asm ("0x007B");

volatile unsigned char U1BR0 __asm ("0x007C");

volatile unsigned char U1BR1 __asm ("0x007D");

volatile unsigned char U1RXBUF __asm ("0x007E");
# 24 "/opt/local/msp430/include/msp430/flash.h" 3
volatile unsigned int FCTL3 __asm ("0x012C");
# 27 "/opt/local/msp430/include/msp430/timera.h" 3
volatile unsigned int TA0CTL __asm ("0x0160");

volatile unsigned int TA0R __asm ("0x0170");
#line 127
#line 118
typedef struct __nesc_unnamed4251 {
  volatile unsigned 
  taifg : 1, 
  taie : 1, 
  taclr : 1, 
  dummy : 1, 
  tamc : 2, 
  taid : 2, 
  tassel : 2;
} __attribute((packed))  tactl_t;
#line 143
#line 129
typedef struct __nesc_unnamed4252 {
  volatile unsigned 
  ccifg : 1, 
  cov : 1, 
  out : 1, 
  cci : 1, 
  ccie : 1, 
  outmod : 3, 
  cap : 1, 
  dummy : 1, 
  scci : 1, 
  scs : 1, 
  ccis : 2, 
  cm : 2;
} __attribute((packed))  tacctl_t;


struct timera_t {
  tactl_t ctl;
  tacctl_t cctl0;
  tacctl_t cctl1;
  tacctl_t cctl2;
  volatile unsigned dummy[4];
  volatile unsigned tar;
  volatile unsigned taccr0;
  volatile unsigned taccr1;
  volatile unsigned taccr2;
};



struct timera_t;
# 26 "/opt/local/msp430/include/msp430/timerb.h" 3
volatile unsigned int TBR __asm ("0x0190");


volatile unsigned int TBCCTL0 __asm ("0x0182");





volatile unsigned int TBCCR0 __asm ("0x0192");
#line 76
#line 64
typedef struct __nesc_unnamed4253 {
  volatile unsigned 
  tbifg : 1, 
  tbie : 1, 
  tbclr : 1, 
  dummy1 : 1, 
  tbmc : 2, 
  tbid : 2, 
  tbssel : 2, 
  dummy2 : 1, 
  tbcntl : 2, 
  tbclgrp : 2;
} __attribute((packed))  tbctl_t;
#line 91
#line 78
typedef struct __nesc_unnamed4254 {
  volatile unsigned 
  ccifg : 1, 
  cov : 1, 
  out : 1, 
  cci : 1, 
  ccie : 1, 
  outmod : 3, 
  cap : 1, 
  clld : 2, 
  scs : 1, 
  ccis : 2, 
  cm : 2;
} __attribute((packed))  tbcctl_t;


struct timerb_t {
  tbctl_t ctl;
  tbcctl_t cctl0;
  tbcctl_t cctl1;
  tbcctl_t cctl2;

  tbcctl_t cctl3;
  tbcctl_t cctl4;
  tbcctl_t cctl5;
  tbcctl_t cctl6;



  volatile unsigned tbr;
  volatile unsigned tbccr0;
  volatile unsigned tbccr1;
  volatile unsigned tbccr2;

  volatile unsigned tbccr3;
  volatile unsigned tbccr4;
  volatile unsigned tbccr5;
  volatile unsigned tbccr6;
};





struct timerb_t;
# 20 "/opt/local/msp430/include/msp430/basic_clock.h" 3
volatile unsigned char DCOCTL __asm ("0x0056");

volatile unsigned char BCSCTL1 __asm ("0x0057");

volatile unsigned char BCSCTL2 __asm ("0x0058");
# 18 "/opt/local/msp430/include/msp430/adc12.h" 3
volatile unsigned int ADC12CTL0 __asm ("0x01A0");

volatile unsigned int ADC12CTL1 __asm ("0x01A2");
#line 42
#line 30
typedef struct __nesc_unnamed4255 {
  volatile unsigned 
  adc12sc : 1, 
  enc : 1, 
  adc12tovie : 1, 
  adc12ovie : 1, 
  adc12on : 1, 
  refon : 1, 
  r2_5v : 1, 
  msc : 1, 
  sht0 : 4, 
  sht1 : 4;
} __attribute((packed))  adc12ctl0_t;
#line 54
#line 44
typedef struct __nesc_unnamed4256 {
  volatile unsigned 
  adc12busy : 1, 
  conseq : 2, 
  adc12ssel : 2, 
  adc12div : 3, 
  issh : 1, 
  shp : 1, 
  shs : 2, 
  cstartadd : 4;
} __attribute((packed))  adc12ctl1_t;
#line 74
#line 56
typedef struct __nesc_unnamed4257 {
  volatile unsigned 
  bit0 : 1, 
  bit1 : 1, 
  bit2 : 1, 
  bit3 : 1, 
  bit4 : 1, 
  bit5 : 1, 
  bit6 : 1, 
  bit7 : 1, 
  bit8 : 1, 
  bit9 : 1, 
  bit10 : 1, 
  bit11 : 1, 
  bit12 : 1, 
  bit13 : 1, 
  bit14 : 1, 
  bit15 : 1;
} __attribute((packed))  adc12xflg_t;


struct adc12_t {
  adc12ctl0_t ctl0;
  adc12ctl1_t ctl1;
  adc12xflg_t ifg;
  adc12xflg_t ie;
  adc12xflg_t iv;
};




struct adc12_t;
# 18 "/opt/local/msp430/include/msp430/dma.h" 3
volatile unsigned int DMACTL0 __asm ("0x0122");



volatile unsigned int DMA0CTL __asm ("0x01E0");







volatile unsigned int DMA1CTL __asm ("0x01E8");







volatile unsigned int DMA2CTL __asm ("0x01F0");
# 65 "/opt/local/msp430/include/msp430x16x.h" 3
volatile unsigned char IFG1 __asm ("0x0002");







volatile unsigned char IE2 __asm ("0x0001");









volatile unsigned char ME1 __asm ("0x0004");





volatile unsigned char ME2 __asm ("0x0005");
# 148 "/Users/jingyuancheng/tinyos/moteiv/tos/platform/msp430/msp430hardware.h"
static volatile uint8_t U0CTLnr __asm ("0x0070");
static volatile uint8_t I2CTCTLnr __asm ("0x0071");
static volatile uint8_t I2CDCTLnr __asm ("0x0072");










static __inline void TOSH_wait(void );
#line 174
static __inline void TOSH_uwait(uint16_t u);
#line 196
static inline void __nesc_disable_interrupt();





static inline void __nesc_enable_interrupt();




static inline bool are_interrupts_enabled();




typedef bool __nesc_atomic_t;

__nesc_atomic_t __nesc_atomic_start(void );
void __nesc_atomic_end(__nesc_atomic_t oldSreg);



__nesc_atomic_t __nesc_atomic_start(void );






void __nesc_atomic_end(__nesc_atomic_t reenable_interrupts);








bool LPMode_disabled = FALSE;









static __inline void __nesc_atomic_sleep();
# 28 "/Users/jingyuancheng/tinyos/moteiv/tinyos-1.x/tos/platform/msp430/MSP430GeneralIO.h"
static inline void TOSH_MAKE_PORT10_INPUT();

static inline uint8_t TOSH_READ_PORT12_PIN();
static inline void TOSH_MAKE_PORT13_INPUT();
static inline void TOSH_MAKE_PORT14_INPUT();
static inline void TOSH_SET_PORT15_PIN();
#line 33
static inline void TOSH_CLR_PORT15_PIN();
#line 33
static inline void TOSH_MAKE_PORT15_OUTPUT();
static inline void TOSH_SET_PORT16_PIN();
#line 34
static inline void TOSH_CLR_PORT16_PIN();
#line 34
static inline void TOSH_MAKE_PORT16_OUTPUT();


static inline void TOSH_SET_PORT20_PIN();
#line 37
static inline void TOSH_CLR_PORT20_PIN();
#line 37
static inline void TOSH_MAKE_PORT20_OUTPUT();
static inline void TOSH_SET_PORT21_PIN();
#line 38
static inline void TOSH_CLR_PORT21_PIN();
#line 38
static inline void TOSH_MAKE_PORT21_OUTPUT();

static inline void TOSH_SET_PORT23_PIN();
#line 40
static inline void TOSH_CLR_PORT23_PIN();
#line 40
static inline void TOSH_MAKE_PORT23_OUTPUT();


static inline void TOSH_SET_PORT26_PIN();
#line 43
static inline void TOSH_CLR_PORT26_PIN();
#line 43
static inline void TOSH_MAKE_PORT26_OUTPUT();
#line 68
static inline void TOSH_SET_PORT54_PIN();
#line 68
static inline void TOSH_CLR_PORT54_PIN();
#line 68
static inline void TOSH_MAKE_PORT54_OUTPUT();
static inline void TOSH_SET_PORT55_PIN();
#line 69
static inline void TOSH_CLR_PORT55_PIN();
#line 69
static inline void TOSH_MAKE_PORT55_OUTPUT();
static inline void TOSH_SET_PORT56_PIN();
#line 70
static inline void TOSH_CLR_PORT56_PIN();
#line 70
static inline void TOSH_MAKE_PORT56_OUTPUT();
# 116 "/Users/jingyuancheng/tinyos/moteiv/tos/platform/msp430/adc/MSP430ADC12.h"
#line 105
typedef struct __nesc_unnamed4258 {

  unsigned int refVolt2_5 : 1;
  unsigned int clockSourceSHT : 2;
  unsigned int clockSourceSAMPCON : 2;
  unsigned int clockDivSAMPCON : 2;
  unsigned int referenceVoltage : 3;
  unsigned int clockDivSHT : 3;
  unsigned int inputChannel : 4;
  unsigned int sampleHoldTime : 4;
  unsigned int  : 0;
} MSP430ADC12Settings_t;






#line 118
typedef enum __nesc_unnamed4259 {

  MSP430ADC12_FAIL = 0, 
  MSP430ADC12_SUCCESS = 1, 
  MSP430ADC12_DELAYED = 2
} msp430ADCresult_t;

enum refVolt2_5_enum {

  REFVOLT_LEVEL_1_5 = 0, 
  REFVOLT_LEVEL_2_5 = 1
};

enum clockDivSHT_enum {

  SHT_CLOCK_DIV_1 = 0, 
  SHT_CLOCK_DIV_2 = 1, 
  SHT_CLOCK_DIV_3 = 2, 
  SHT_CLOCK_DIV_4 = 3, 
  SHT_CLOCK_DIV_5 = 4, 
  SHT_CLOCK_DIV_6 = 5, 
  SHT_CLOCK_DIV_7 = 6, 
  SHT_CLOCK_DIV_8 = 7
};

enum clockDivSAMPCON_enum {

  SAMPCON_CLOCK_DIV_1 = 0, 
  SAMPCON_CLOCK_DIV_2 = 1, 
  SAMPCON_CLOCK_DIV_3 = 2, 
  SAMPCON_CLOCK_DIV_4 = 3
};

enum clockSourceSAMPCON_enum {

  SAMPCON_SOURCE_TACLK = 0, 
  SAMPCON_SOURCE_ACLK = 1, 
  SAMPCON_SOURCE_SMCLK = 2, 
  SAMPCON_SOURCE_INCLK = 3
};

enum inputChannel_enum {


  INPUT_CHANNEL_A0 = 0, 
  INPUT_CHANNEL_A1 = 1, 
  INPUT_CHANNEL_A2 = 2, 
  INPUT_CHANNEL_A3 = 3, 
  INPUT_CHANNEL_A4 = 4, 
  INPUT_CHANNEL_A5 = 5, 
  INPUT_CHANNEL_A6 = 6, 
  INPUT_CHANNEL_A7 = 7, 
  EXTERNAL_REFERENCE_VOLTAGE = 8, 
  REFERENCE_VOLTAGE_NEGATIVE_TERMINAL = 9, 
  INTERNAL_TEMPERATURE = 10, 
  INTERNAL_VOLTAGE = 11
};

enum referenceVoltage_enum {

  REFERENCE_AVcc_AVss = 0, 
  REFERENCE_VREFplus_AVss = 1, 
  REFERENCE_VeREFplus_AVss = 2, 
  REFERENCE_AVcc_VREFnegterm = 4, 
  REFERENCE_VREFplus_VREFnegterm = 5, 
  REFERENCE_VeREFplus_VREFnegterm = 6
};

enum clockSourceSHT_enum {

  SHT_SOURCE_ADC12OSC = 0, 
  SHT_SOURCE_ACLK = 1, 
  SHT_SOURCE_MCLK = 2, 
  SHT_SOURCE_SMCLK = 3
};

enum sampleHold_enum {

  SAMPLE_HOLD_4_CYCLES = 0, 
  SAMPLE_HOLD_8_CYCLES = 1, 
  SAMPLE_HOLD_16_CYCLES = 2, 
  SAMPLE_HOLD_32_CYCLES = 3, 
  SAMPLE_HOLD_64_CYCLES = 4, 
  SAMPLE_HOLD_96_CYCLES = 5, 
  SAMPLE_HOLD_123_CYCLES = 6, 
  SAMPLE_HOLD_192_CYCLES = 7, 
  SAMPLE_HOLD_256_CYCLES = 8, 
  SAMPLE_HOLD_384_CYCLES = 9, 
  SAMPLE_HOLD_512_CYCLES = 10, 
  SAMPLE_HOLD_768_CYCLES = 11, 
  SAMPLE_HOLD_1024_CYCLES = 12
};









#line 216
typedef union __nesc_unnamed4260 {
  uint32_t i;
  MSP430ADC12Settings_t s;
} MSP430ADC12Settings_ut;


static __inline MSP430ADC12Settings_t int2adcSettings(uint32_t i);





enum __nesc_unnamed4261 {

  ADC_IDLE = 0x00, 
  SINGLE_CHANNEL = 0x01, 
  REPEAT_SINGLE_CHANNEL = 0x02, 
  SEQUENCE_OF_CHANNELS = 0x04, 
  REPEAT_SEQUENCE_OF_CHANNELS = 0x08, 
  TIMER_USED = 0x10, 
  RESERVED = 0x20, 
  VREF_WAIT = 0x40, 
  ADC_INTERRUPT_DISABLED = 0x80
};
#line 262
#line 256
typedef struct __nesc_unnamed4262 {

  volatile unsigned 
  inch : 4, 
  sref : 3, 
  eos : 1;
} __attribute((packed))  adc12memctl_t;
#line 275
#line 264
typedef struct __nesc_unnamed4263 {

  unsigned int refVolt2_5 : 1;
  unsigned int gotRefVolt : 1;
  unsigned int result_16bit : 1;
  unsigned int clockSourceSHT : 2;
  unsigned int clockSourceSAMPCON : 2;
  unsigned int clockDivSAMPCON : 2;
  unsigned int clockDivSHT : 3;
  unsigned int sampleHoldTime : 4;
  adc12memctl_t memctl;
} __attribute((packed))  adc12settings_t;
# 49 "/Users/jingyuancheng/tinyos/moteiv/tos/lib/CC2420Radio/CC2420Const.h"
#line 45
typedef enum __nesc_unnamed4264 {
  CC2420_SUCCESS = 0, 
  CC2420_E_SHUTDOWN, 
  CC2420_E_UNKNOWN
} cc2420_error_t;





#line 51
typedef enum __nesc_unnamed4265 {
  CC2420_LINKSTATE_OFF = 0, 
  CC2420_LINKSTATE_ON, 
  CC2420_LINKSTATE_WARMUP
} cc2420_linkstate_t;


enum __nesc_unnamed4266 {
  CC2420_TIME_BIT = 4, 
  CC2420_TIME_BYTE = CC2420_TIME_BIT << 3, 
  CC2420_TIME_SYMBOL = 16
};










uint8_t CC2420_CHANNEL = 11;
uint8_t CC2420_RFPOWER = 0x1F;

enum __nesc_unnamed4267 {
  CC2420_MIN_CHANNEL = 11, 
  CC2420_MAX_CHANNEL = 26
};
#line 275
enum __nesc_unnamed4268 {
  CP_MAIN = 0, 
  CP_MDMCTRL0, 
  CP_MDMCTRL1, 
  CP_RSSI, 
  CP_SYNCWORD, 
  CP_TXCTRL, 
  CP_RXCTRL0, 
  CP_RXCTRL1, 
  CP_FSCTRL, 
  CP_SECCTRL0, 
  CP_SECCTRL1, 
  CP_BATTMON, 
  CP_IOCFG0, 
  CP_IOCFG1
};
# 38 "/Users/jingyuancheng/tinyos/moteiv/tos/lib/CC2420Radio/AM.h"
enum __nesc_unnamed4269 {
  TOS_BCAST_ADDR = 0xffff, 
  TOS_UART_ADDR = 0x007e
};





enum __nesc_unnamed4270 {
  TOS_DEFAULT_AM_GROUP = 0x7d
};

uint8_t TOS_AM_GROUP = TOS_DEFAULT_AM_GROUP;
#line 92
#line 68
typedef struct TOS_Msg {


  uint8_t length;
  uint8_t fcfhi;
  uint8_t fcflo;
  uint8_t dsn;
  uint16_t destpan;
  uint16_t addr;
  uint8_t type;
  uint8_t group;
  int8_t data[66];







  uint8_t strength;
  uint8_t lqi;
  bool crc;
  bool ack;
  uint32_t time;
} __attribute((packed))  TOS_Msg;

enum __nesc_unnamed4271 {

  MSG_HEADER_SIZE = (size_t )& ((struct TOS_Msg *)0)->data - 1, 

  MSG_FOOTER_SIZE = 2, 

  MSG_DATA_SIZE = (size_t )& ((struct TOS_Msg *)0)->strength + sizeof(uint16_t ), 

  DATA_LENGTH = 66, 

  LENGTH_BYTE_NUMBER = (size_t )& ((struct TOS_Msg *)0)->length + 1
};

typedef TOS_Msg *TOS_MsgPtr;
# 29 "/Users/jingyuancheng/tinyos/moteiv/tos/lib/timer/Timer2.h"
typedef struct __nesc_unnamed4272 {
} 
#line 29
TMilli;
typedef struct __nesc_unnamed4273 {
} 
#line 30
T32khz;
typedef struct __nesc_unnamed4274 {
} 
#line 31
TMicro;
# 16 "/Users/jingyuancheng/tinyos/moteiv/tos/lib/resource/Resource.h"
enum __nesc_unnamed4275 {
  RESOURCE_NONE = 0
};
# 20 "/Users/jingyuancheng/tinyos/moteiv/tos/platform/tmote/hardware.h"
static inline void TOSH_SET_RADIO_CSN_PIN();
#line 20
static inline void TOSH_CLR_RADIO_CSN_PIN();
#line 20
static inline void TOSH_MAKE_RADIO_CSN_OUTPUT();



static inline void TOSH_MAKE_RADIO_SFD_INPUT();



static inline uint8_t TOSH_READ_RADIO_CCA_PIN();

static inline uint8_t TOSH_READ_CC_FIFOP_PIN();
static inline uint8_t TOSH_READ_CC_FIFO_PIN();
static inline uint8_t TOSH_READ_CC_SFD_PIN();
#line 32
static inline void TOSH_SEL_CC_SFD_MODFUNC();
#line 32
static inline void TOSH_SEL_CC_SFD_IOFUNC();
static inline void TOSH_SET_CC_VREN_PIN();
#line 33
static inline void TOSH_CLR_CC_VREN_PIN();
static inline void TOSH_SET_CC_RSTN_PIN();
#line 34
static inline void TOSH_CLR_CC_RSTN_PIN();


static inline void TOSH_SEL_SOMI0_MODFUNC();
#line 37
static inline void TOSH_SEL_SOMI0_IOFUNC();
static inline void TOSH_SET_SIMO0_PIN();
#line 38
static inline void TOSH_CLR_SIMO0_PIN();
#line 38
static inline void TOSH_MAKE_SIMO0_OUTPUT();
#line 38
static inline void TOSH_SEL_SIMO0_MODFUNC();
#line 38
static inline void TOSH_SEL_SIMO0_IOFUNC();
static inline void TOSH_SET_UCLK0_PIN();
#line 39
static inline void TOSH_CLR_UCLK0_PIN();
#line 39
static inline void TOSH_MAKE_UCLK0_OUTPUT();
#line 39
static inline void TOSH_SEL_UCLK0_MODFUNC();
#line 39
static inline void TOSH_SEL_UCLK0_IOFUNC();

static inline void TOSH_CLR_UTXD0_PIN();
#line 41
static inline void TOSH_SEL_UTXD0_IOFUNC();
static inline void TOSH_SEL_URXD0_IOFUNC();
static inline void TOSH_SEL_UTXD1_MODFUNC();
#line 43
static inline void TOSH_SEL_UTXD1_IOFUNC();
static inline void TOSH_SEL_URXD1_MODFUNC();
#line 44
static inline void TOSH_SEL_URXD1_IOFUNC();
static inline void TOSH_SEL_UCLK1_IOFUNC();
static inline void TOSH_SEL_SOMI1_IOFUNC();
static inline void TOSH_SEL_SIMO1_IOFUNC();
#line 76
static inline void TOSH_SET_FLASH_CS_PIN();
#line 76
static inline void TOSH_CLR_FLASH_CS_PIN();
#line 76
static inline void TOSH_MAKE_FLASH_CS_OUTPUT();
static inline void TOSH_SET_FLASH_HOLD_PIN();
#line 77
static inline void TOSH_MAKE_FLASH_HOLD_OUTPUT();










static void TOSH_FLASH_M25P_DP_bit(bool set);










static inline void TOSH_FLASH_M25P_DP();
#line 143
enum __nesc_unnamed4276 {
  MSP430_INIT_PIN_SELBIT = 0x04, 
  MSP430_INIT_PIN_DIRBIT = 0x02, 
  MSP430_INIT_PIN_OUTBIT = 0x01
};




enum __nesc_unnamed4277 {
  MSP430_INIT_PIN_DIGITAL_OUT_0 = MSP430_INIT_PIN_DIRBIT, 
  MSP430_INIT_PIN_DIGITAL_OUT_1 = MSP430_INIT_PIN_DIRBIT | MSP430_INIT_PIN_OUTBIT, 
  MSP430_INIT_PIN_DIGITAL_IN = 0, 
  MSP430_INIT_PIN_MODULE_IN = MSP430_INIT_PIN_SELBIT, 
  MSP430_INIT_PIN_MODULE_OUT = MSP430_INIT_PIN_SELBIT | MSP430_INIT_PIN_DIRBIT
};


enum __nesc_unnamed4278 {










  MSP430_INIT_P10 = MSP430_INIT_PIN_DIGITAL_OUT_0, 
  MSP430_INIT_P11 = MSP430_INIT_PIN_DIGITAL_IN, 
  MSP430_INIT_P12 = MSP430_INIT_PIN_DIGITAL_IN, 
  MSP430_INIT_P13 = MSP430_INIT_PIN_DIGITAL_OUT_0, 
  MSP430_INIT_P14 = MSP430_INIT_PIN_DIGITAL_OUT_0, 
  MSP430_INIT_P15 = MSP430_INIT_PIN_DIGITAL_OUT_0, 
  MSP430_INIT_P16 = MSP430_INIT_PIN_DIGITAL_OUT_0, 
  MSP430_INIT_P17 = MSP430_INIT_PIN_DIGITAL_OUT_0, 
#line 191
  MSP430_INIT_P20 = MSP430_INIT_PIN_DIGITAL_OUT_0, 
  MSP430_INIT_P21 = MSP430_INIT_PIN_DIGITAL_OUT_0, 
  MSP430_INIT_P22 = MSP430_INIT_PIN_DIGITAL_IN, 
  MSP430_INIT_P23 = MSP430_INIT_PIN_DIGITAL_OUT_0, 
  MSP430_INIT_P24 = MSP430_INIT_PIN_DIGITAL_OUT_1, 
  MSP430_INIT_P25 = MSP430_INIT_PIN_DIGITAL_OUT_1, 
  MSP430_INIT_P26 = MSP430_INIT_PIN_DIGITAL_OUT_0, 
  MSP430_INIT_P27 = MSP430_INIT_PIN_DIGITAL_IN, 
#line 210
  MSP430_INIT_P30 = MSP430_INIT_PIN_DIGITAL_OUT_0, 
  MSP430_INIT_P31 = MSP430_INIT_PIN_DIGITAL_OUT_1, 
  MSP430_INIT_P32 = MSP430_INIT_PIN_DIGITAL_OUT_1, 
  MSP430_INIT_P33 = MSP430_INIT_PIN_DIGITAL_OUT_0, 
  MSP430_INIT_P34 = MSP430_INIT_PIN_DIGITAL_OUT_0, 
  MSP430_INIT_P35 = MSP430_INIT_PIN_DIGITAL_OUT_0, 
  MSP430_INIT_P36 = MSP430_INIT_PIN_DIGITAL_OUT_0, 
  MSP430_INIT_P37 = MSP430_INIT_PIN_DIGITAL_OUT_0, 
#line 229
  MSP430_INIT_P40 = MSP430_INIT_PIN_DIGITAL_OUT_0, 
  MSP430_INIT_P41 = MSP430_INIT_PIN_DIGITAL_OUT_0, 
  MSP430_INIT_P42 = MSP430_INIT_PIN_DIGITAL_OUT_1, 
  MSP430_INIT_P43 = MSP430_INIT_PIN_DIGITAL_OUT_0, 
  MSP430_INIT_P44 = MSP430_INIT_PIN_DIGITAL_OUT_1, 
  MSP430_INIT_P45 = MSP430_INIT_PIN_DIGITAL_OUT_0, 
  MSP430_INIT_P46 = MSP430_INIT_PIN_DIGITAL_OUT_1, 
  MSP430_INIT_P47 = MSP430_INIT_PIN_DIGITAL_OUT_1, 
#line 248
  MSP430_INIT_P50 = MSP430_INIT_PIN_DIGITAL_OUT_0, 
  MSP430_INIT_P51 = MSP430_INIT_PIN_DIGITAL_OUT_0, 
  MSP430_INIT_P52 = MSP430_INIT_PIN_DIGITAL_OUT_0, 
  MSP430_INIT_P53 = MSP430_INIT_PIN_DIGITAL_OUT_0, 
  MSP430_INIT_P54 = MSP430_INIT_PIN_DIGITAL_OUT_1, 
  MSP430_INIT_P55 = MSP430_INIT_PIN_DIGITAL_OUT_1, 
  MSP430_INIT_P56 = MSP430_INIT_PIN_DIGITAL_OUT_1, 
  MSP430_INIT_P57 = MSP430_INIT_PIN_DIGITAL_OUT_0, 
#line 267
  MSP430_INIT_P60 = MSP430_INIT_PIN_DIGITAL_OUT_0, 
  MSP430_INIT_P61 = MSP430_INIT_PIN_DIGITAL_OUT_0, 
  MSP430_INIT_P62 = MSP430_INIT_PIN_DIGITAL_OUT_0, 
  MSP430_INIT_P63 = MSP430_INIT_PIN_DIGITAL_OUT_0, 
  MSP430_INIT_P64 = MSP430_INIT_PIN_DIGITAL_OUT_0, 
  MSP430_INIT_P65 = MSP430_INIT_PIN_DIGITAL_OUT_0, 
  MSP430_INIT_P66 = MSP430_INIT_PIN_DIGITAL_OUT_0, 
  MSP430_INIT_P67 = MSP430_INIT_PIN_DIGITAL_OUT_0
};



static inline void TOSH_SET_PIN_DIRECTIONS(void );
# 54 "/Users/jingyuancheng/tinyos/moteiv/tinyos-1.x/tos/types/dbg_modes.h"
typedef long long TOS_dbg_mode;



enum __nesc_unnamed4279 {
  DBG_ALL = ~0ULL, 


  DBG_BOOT = 1ULL << 0, 
  DBG_CLOCK = 1ULL << 1, 
  DBG_TASK = 1ULL << 2, 
  DBG_SCHED = 1ULL << 3, 
  DBG_SENSOR = 1ULL << 4, 
  DBG_LED = 1ULL << 5, 
  DBG_CRYPTO = 1ULL << 6, 


  DBG_ROUTE = 1ULL << 7, 
  DBG_AM = 1ULL << 8, 
  DBG_CRC = 1ULL << 9, 
  DBG_PACKET = 1ULL << 10, 
  DBG_ENCODE = 1ULL << 11, 
  DBG_RADIO = 1ULL << 12, 


  DBG_LOG = 1ULL << 13, 
  DBG_ADC = 1ULL << 14, 
  DBG_I2C = 1ULL << 15, 
  DBG_UART = 1ULL << 16, 
  DBG_PROG = 1ULL << 17, 
  DBG_SOUNDER = 1ULL << 18, 
  DBG_TIME = 1ULL << 19, 
  DBG_POWER = 1ULL << 20, 



  DBG_SIM = 1ULL << 21, 
  DBG_QUEUE = 1ULL << 22, 
  DBG_SIMRADIO = 1ULL << 23, 
  DBG_HARD = 1ULL << 24, 
  DBG_MEM = 1ULL << 25, 



  DBG_USR1 = 1ULL << 27, 
  DBG_USR2 = 1ULL << 28, 
  DBG_USR3 = 1ULL << 29, 
  DBG_TEMP = 1ULL << 30, 

  DBG_ERROR = 1ULL << 31, 
  DBG_NONE = 0, 

  DBG_DEFAULT = DBG_ALL
};
# 8 "/Users/jingyuancheng/tinyos/moteiv/tos/lib/sched/sched.c"
__nesc_atomic_t __nesc_atomic_start(void )  ;
void __nesc_atomic_end(__nesc_atomic_t reenable_interrupts)  ;
# 164 "/Users/jingyuancheng/tinyos/moteiv/tos/system/tos.h"
static inline void *nmemset(void *to, int val, size_t n);
# 28 "/Users/jingyuancheng/tinyos/moteiv/tinyos-1.x/tos/system/Ident.h"
enum __nesc_unnamed4280 {

  IDENT_MAX_PROGRAM_NAME_LENGTH = 16
};






#line 33
typedef struct __nesc_unnamed4281 {

  uint32_t unix_time;
  uint32_t user_hash;
  char program_name[IDENT_MAX_PROGRAM_NAME_LENGTH];
} Ident_t;
# 43 "/opt/local/lib/gcc-lib/msp430/3.2.3/include/stdarg.h" 3
typedef __builtin_va_list __gnuc_va_list;
#line 110
typedef __gnuc_va_list va_list;
# 25 "./ADC8IO14.h"
#line 22
typedef struct ADCMsg {
  uint8_t data[8 * 8];
  uint16_t count;
} ADCMsg_t;
# 28 "/Users/jingyuancheng/tinyos/moteiv/tos/platform/msp430/timer/MSP430Timer.h"
enum __nesc_unnamed4282 {
  MSP430TIMER_CM_NONE = 0, 
  MSP430TIMER_CM_RISING = 1, 
  MSP430TIMER_CM_FALLING = 2, 
  MSP430TIMER_CM_BOTH = 3, 

  MSP430TIMER_STOP_MODE = 0, 
  MSP430TIMER_UP_MODE = 1, 
  MSP430TIMER_CONTINUOUS_MODE = 2, 
  MSP430TIMER_UPDOWN_MODE = 3, 

  MSP430TIMER_TACLK = 0, 
  MSP430TIMER_TBCLK = 0, 
  MSP430TIMER_ACLK = 1, 
  MSP430TIMER_SMCLK = 2, 
  MSP430TIMER_INCLK = 3, 

  MSP430TIMER_CLOCKDIV_1 = 0, 
  MSP430TIMER_CLOCKDIV_2 = 1, 
  MSP430TIMER_CLOCKDIV_4 = 2, 
  MSP430TIMER_CLOCKDIV_8 = 3
};
#line 64
#line 51
typedef struct __nesc_unnamed4283 {

  int ccifg : 1;
  int cov : 1;
  int out : 1;
  int cci : 1;
  int ccie : 1;
  int outmod : 3;
  int cap : 1;
  int clld : 2;
  int scs : 1;
  int ccis : 2;
  int cm : 2;
} MSP430CompareControl_t;
#line 76
#line 66
typedef struct __nesc_unnamed4284 {

  int taifg : 1;
  int taie : 1;
  int taclr : 1;
  int _unused0 : 1;
  int mc : 2;
  int id : 2;
  int tassel : 2;
  int _unused1 : 6;
} MSP430TimerAControl_t;
#line 91
#line 78
typedef struct __nesc_unnamed4285 {

  int tbifg : 1;
  int tbie : 1;
  int tbclr : 1;
  int _unused0 : 1;
  int mc : 2;
  int id : 2;
  int tbssel : 2;
  int _unused1 : 1;
  int cntl : 2;
  int tbclgrp : 2;
  int _unused2 : 1;
} MSP430TimerBControl_t;
# 20 "/Users/jingyuancheng/tinyos/moteiv/tos/lib/util/reservedQueue.h"
#line 16
typedef struct __nesc_unnamed4286 {
  uint8_t head;
  uint8_t tail;
  uint8_t next[0];
} ReservedQueue_t;

enum __nesc_unnamed4287 {
  RQUEUE_NONE = 255
};

static void rqueue_init(ReservedQueue_t *q, uint8_t count);
#line 43
static bool rqueue_isQueued(ReservedQueue_t *q, uint8_t id);




static bool rqueue_remove(ReservedQueue_t *q, uint8_t id);
#line 92
static inline uint8_t rqueue_pop(ReservedQueue_t *q);






static bool rqueue_push_priv(ReservedQueue_t *q, uint8_t id, bool second);
#line 124
static inline bool rqueue_push(ReservedQueue_t *q, uint8_t id);









static inline bool rqueue_pushFront(ReservedQueue_t *q, uint8_t id);
# 45 "/Users/jingyuancheng/tinyos/moteiv/tos/lib/timer/Timer.h"
enum __nesc_unnamed4288 {
  TIMER_REPEAT = 0, 
  TIMER_ONE_SHOT = 1, 
  NUM_TIMERS = 5U
};
# 38 "/Users/jingyuancheng/tinyos/moteiv/tos/lib/sp/sp.h"
#line 34
typedef enum __nesc_unnamed4289 {
  SP_I_NOT_SPECIFIED = 0, 
  SP_I_RADIO = 0U, 
  SP_I_UART = 1U
} sp_interface_t;

typedef uint16_t sp_address_t;
typedef uint8_t sp_device_t;
#line 53
#line 43
typedef struct SPMessage {
  TOS_Msg *msg;
  uint32_t time;
  sp_address_t addr;
  sp_device_t dev;
  uint8_t id;
  uint8_t quantity;
  uint8_t flags;
  uint8_t retries;
  uint8_t length;
} sp_message_t;







#line 55
typedef struct SPNeighborTableEntry {
  uint16_t addr;
  uint32_t timeon;
  uint32_t timeoff;
  uint8_t id;
  uint8_t flags;
} sp_neighbor_t;






#line 63
typedef enum __nesc_unnamed4290 {
  SP_RADIO_OFF = 0, 
  SP_RADIO_ON, 
  SP_RADIO_WAKEUP, 
  SP_RADIO_SHUTDOWN
} sp_linkstate_t;
#line 91
#line 70
typedef enum __nesc_unnamed4291 {

  SP_FLAG_C_TIMESTAMP = 0x001, 
  SP_FLAG_C_RELIABLE = 0x002, 
  SP_FLAG_C_URGENT = 0x004, 
  SP_FLAG_C_NONE = 0x000, 
  SP_FLAG_C_ALL = 0x007, 

  SP_FLAG_F_CONGESTION = 0x010, 
  SP_FLAG_F_PHASE = 0x020, 
  SP_FLAG_F_RELIABLE = 0x040, 
  SP_FLAG_F_NONE = 0x000, 
  SP_FLAG_F_ALL = 0x070, 


  SP_FLAG_C_BUSY = 0x080, 
  SP_FLAG_C_FUTURES = 0x008
} 



sp_message_flags_t;







#line 93
typedef enum __nesc_unnamed4292 {
  SP_FLAG_TABLE = 0x01, 
  SP_FLAG_BUSY = 0x02, 
  SP_FLAG_LISTEN = 0x04, 
  SP_FLAG_LINK_STARTED = 0x08, 
  SP_FLAG_LINK_ACTIVE = 0x10
} sp_neighbor_flags_t;







#line 101
typedef enum __nesc_unnamed4293 {
  SP_SUCCESS = 0, 
  SP_E_UNKNOWN = 0x01, 
  SP_E_RELIABLE = 0x02, 
  SP_E_BUF_UNDERRUN = 0x03, 
  SP_E_SHUTDOWN = 0x04
} sp_error_t;
# 39 "/Users/jingyuancheng/tinyos/moteiv/tinyos-1.x/tos/platform/msp430/msp430usart.h"
#line 31
typedef enum __nesc_unnamed4294 {

  USART_NONE = 0, 
  USART_UART = 1, 
  USART_UART_TX = 2, 
  USART_UART_RX = 3, 
  USART_SPI = 4, 
  USART_I2C = 5
} msp430_usartmode_t;
# 34 "/Users/jingyuancheng/tinyos/moteiv/tos/platform/msp430/crc.h"
static const uint16_t ccitt_crc16_table[256] = { 
0x0000, 0x1021, 0x2042, 0x3063, 0x4084, 0x50a5, 0x60c6, 0x70e7, 
0x8108, 0x9129, 0xa14a, 0xb16b, 0xc18c, 0xd1ad, 0xe1ce, 0xf1ef, 
0x1231, 0x0210, 0x3273, 0x2252, 0x52b5, 0x4294, 0x72f7, 0x62d6, 
0x9339, 0x8318, 0xb37b, 0xa35a, 0xd3bd, 0xc39c, 0xf3ff, 0xe3de, 
0x2462, 0x3443, 0x0420, 0x1401, 0x64e6, 0x74c7, 0x44a4, 0x5485, 
0xa56a, 0xb54b, 0x8528, 0x9509, 0xe5ee, 0xf5cf, 0xc5ac, 0xd58d, 
0x3653, 0x2672, 0x1611, 0x0630, 0x76d7, 0x66f6, 0x5695, 0x46b4, 
0xb75b, 0xa77a, 0x9719, 0x8738, 0xf7df, 0xe7fe, 0xd79d, 0xc7bc, 
0x48c4, 0x58e5, 0x6886, 0x78a7, 0x0840, 0x1861, 0x2802, 0x3823, 
0xc9cc, 0xd9ed, 0xe98e, 0xf9af, 0x8948, 0x9969, 0xa90a, 0xb92b, 
0x5af5, 0x4ad4, 0x7ab7, 0x6a96, 0x1a71, 0x0a50, 0x3a33, 0x2a12, 
0xdbfd, 0xcbdc, 0xfbbf, 0xeb9e, 0x9b79, 0x8b58, 0xbb3b, 0xab1a, 
0x6ca6, 0x7c87, 0x4ce4, 0x5cc5, 0x2c22, 0x3c03, 0x0c60, 0x1c41, 
0xedae, 0xfd8f, 0xcdec, 0xddcd, 0xad2a, 0xbd0b, 0x8d68, 0x9d49, 
0x7e97, 0x6eb6, 0x5ed5, 0x4ef4, 0x3e13, 0x2e32, 0x1e51, 0x0e70, 
0xff9f, 0xefbe, 0xdfdd, 0xcffc, 0xbf1b, 0xaf3a, 0x9f59, 0x8f78, 
0x9188, 0x81a9, 0xb1ca, 0xa1eb, 0xd10c, 0xc12d, 0xf14e, 0xe16f, 
0x1080, 0x00a1, 0x30c2, 0x20e3, 0x5004, 0x4025, 0x7046, 0x6067, 
0x83b9, 0x9398, 0xa3fb, 0xb3da, 0xc33d, 0xd31c, 0xe37f, 0xf35e, 
0x02b1, 0x1290, 0x22f3, 0x32d2, 0x4235, 0x5214, 0x6277, 0x7256, 
0xb5ea, 0xa5cb, 0x95a8, 0x8589, 0xf56e, 0xe54f, 0xd52c, 0xc50d, 
0x34e2, 0x24c3, 0x14a0, 0x0481, 0x7466, 0x6447, 0x5424, 0x4405, 
0xa7db, 0xb7fa, 0x8799, 0x97b8, 0xe75f, 0xf77e, 0xc71d, 0xd73c, 
0x26d3, 0x36f2, 0x0691, 0x16b0, 0x6657, 0x7676, 0x4615, 0x5634, 
0xd94c, 0xc96d, 0xf90e, 0xe92f, 0x99c8, 0x89e9, 0xb98a, 0xa9ab, 
0x5844, 0x4865, 0x7806, 0x6827, 0x18c0, 0x08e1, 0x3882, 0x28a3, 
0xcb7d, 0xdb5c, 0xeb3f, 0xfb1e, 0x8bf9, 0x9bd8, 0xabbb, 0xbb9a, 
0x4a75, 0x5a54, 0x6a37, 0x7a16, 0x0af1, 0x1ad0, 0x2ab3, 0x3a92, 
0xfd2e, 0xed0f, 0xdd6c, 0xcd4d, 0xbdaa, 0xad8b, 0x9de8, 0x8dc9, 
0x7c26, 0x6c07, 0x5c64, 0x4c45, 0x3ca2, 0x2c83, 0x1ce0, 0x0cc1, 
0xef1f, 0xff3e, 0xcf5d, 0xdf7c, 0xaf9b, 0xbfba, 0x8fd9, 0x9ff8, 
0x6e17, 0x7e36, 0x4e55, 0x5e74, 0x2e93, 0x3eb2, 0x0ed1, 0x1ef0 };


static inline uint16_t crcByte(uint16_t fcs, uint8_t c);
# 4 "/Users/jingyuancheng/tinyos/moteiv/tos/platform/tmote/util/uartdetect/UartDetectMsg.h"
enum __nesc_unnamed4295 {
  AM_UARTDETECTMSG = 101
};

enum __nesc_unnamed4296 {
  UARTDETECT_REQUEST = 0, 
  UARTDETECT_RESPONSE = 1, 
  UARTDETECT_KEEPALIVE = 2
};

enum __nesc_unnamed4297 {
  UARTDETECT_POLL = 1024 * 3
};






#line 18
typedef struct UartDetectMsg {
  uint8_t cmd;
  uint8_t id;
  uint16_t addr;
  uint32_t timeout;
} uartdetectmsg_t;
# 19 "/Users/jingyuancheng/tinyos/moteiv/tinyos-1.x/tos/platform/msp430/msp430baudrates.h"
enum __nesc_unnamed4298 {

  UBR_ACLK_1200 = 0x001B, UMCTL_ACLK_1200 = 0x94, 
  UBR_ACLK_1800 = 0x0012, UMCTL_ACLK_1800 = 0x84, 
  UBR_ACLK_2400 = 0x000D, UMCTL_ACLK_2400 = 0x6D, 
  UBR_ACLK_4800 = 0x0006, UMCTL_ACLK_4800 = 0x77, 
  UBR_ACLK_9600 = 0x0003, UMCTL_ACLK_9600 = 0x29, 


  UBR_SMCLK_1200 = 0x0369, UMCTL_SMCLK_1200 = 0x7B, 
  UBR_SMCLK_1800 = 0x0246, UMCTL_SMCLK_1800 = 0x55, 
  UBR_SMCLK_2400 = 0x01B4, UMCTL_SMCLK_2400 = 0xDF, 
  UBR_SMCLK_4800 = 0x00DA, UMCTL_SMCLK_4800 = 0xAA, 
  UBR_SMCLK_9600 = 0x006D, UMCTL_SMCLK_9600 = 0x44, 
  UBR_SMCLK_19200 = 0x0036, UMCTL_SMCLK_19200 = 0xB5, 
  UBR_SMCLK_38400 = 0x001B, UMCTL_SMCLK_38400 = 0x94, 
  UBR_SMCLK_57600 = 0x0012, UMCTL_SMCLK_57600 = 0x84, 
  UBR_SMCLK_76800 = 0x000D, UMCTL_SMCLK_76800 = 0x6D, 
  UBR_SMCLK_115200 = 0x0009, UMCTL_SMCLK_115200 = 0x10, 
  UBR_SMCLK_230400 = 0x0004, UMCTL_SMCLK_230400 = 0x55, 
  UBR_SMCLK_262144 = 4, UMCTL_SMCLK_262144 = 0
};
# 30 "/Users/jingyuancheng/tinyos/moteiv/tos/lib/sp/cc2420/sp_cc2420.h"
typedef uint16_t radio_addr_t;
typedef uint16_t uart_addr_t;
# 12 "/Users/jingyuancheng/tinyos/moteiv/tos/lib/CC2420Radio/byteorder.h"
static __inline int is_host_lsb();





static __inline uint16_t toLSB16(uint16_t a);




static __inline uint16_t fromLSB16(uint16_t a);
# 36 "/Users/jingyuancheng/tinyos/moteiv/tos/lib/util/circularQueue.h"
typedef uint8_t CircularQueueIndex_t;






#line 38
typedef struct __nesc_unnamed4299 {

  CircularQueueIndex_t front;
  CircularQueueIndex_t back;
  CircularQueueIndex_t size;
} CircularQueue_t;





static void cqueue_init(CircularQueue_t *cq, CircularQueueIndex_t size);
#line 63
static inline CircularQueueIndex_t cqueue_privDec(CircularQueue_t *cq, CircularQueueIndex_t n);





static bool cqueue_isEmpty(CircularQueue_t *cq);
#line 114
static inline result_t cqueue_pushBack(CircularQueue_t *cq);
#line 137
static inline result_t cqueue_popFront(CircularQueue_t *cq);
# 23 "/Users/jingyuancheng/tinyos/moteiv/tos/platform/msp430/adc/RefVolt.h"
#line 18
typedef enum __nesc_unnamed4300 {

  REFERENCE_1_5V, 
  REFERENCE_2_5V, 
  REFERENCE_UNSTABLE
} RefVolt_t;
enum /*MSP430DCOCalibC.MSP430ResourceTimerAC*/MSP430ResourceTimerAC$0$__nesc_unnamed4301 {
  MSP430ResourceTimerAC$0$ID = 0U + 1
};
enum MSP430Timer32khzMapC$__nesc_unnamed4302 {
  MSP430Timer32khzMapC$B0 = 0U
};
enum MSP430Timer32khzMapC$__nesc_unnamed4303 {
  MSP430Timer32khzMapC$B2 = 1U
};
enum MSP430Timer32khzMapC$__nesc_unnamed4304 {
  MSP430Timer32khzMapC$B3 = 2U
};
enum MSP430Timer32khzMapC$__nesc_unnamed4305 {
  MSP430Timer32khzMapC$B4 = 3U
};
enum MSP430Timer32khzMapC$__nesc_unnamed4306 {
  MSP430Timer32khzMapC$B5 = 4U
};
enum MSP430Timer32khzMapC$__nesc_unnamed4307 {
  MSP430Timer32khzMapC$B6 = 5U
};
enum /*MSP430DCOCalibC.MSP430Timer32khzC*/MSP430Timer32khzC$0$__nesc_unnamed4308 {
  MSP430Timer32khzC$0$ALARM_ID = 0U
};
typedef TMilli ADC8IO14P$Timer$precision_tag;
enum /*MainTimerMilliC.MainControlC*/MainControlC$0$__nesc_unnamed4309 {
  MainControlC$0$ID = 0U
};
enum HalTimerMilliC$__nesc_unnamed4310 {
  HalTimerMilliC$NUM_TIMERS = 5U + 0U
};
enum /*HalTimerMilliC.AlarmMilliC.MSP430Timer*/MSP430Timer32khzC$1$__nesc_unnamed4311 {
  MSP430Timer32khzC$1$ALARM_ID = 1U
};
typedef T32khz /*HalTimerMilliC.AlarmMilliC.MSP430Alarm*/MSP430AlarmC$0$frequency_tag;
typedef /*HalTimerMilliC.AlarmMilliC.MSP430Alarm*/MSP430AlarmC$0$frequency_tag /*HalTimerMilliC.AlarmMilliC.MSP430Alarm*/MSP430AlarmC$0$Alarm$precision_tag;
typedef uint16_t /*HalTimerMilliC.AlarmMilliC.MSP430Alarm*/MSP430AlarmC$0$Alarm$size_type;
typedef TMilli /*HalTimerMilliC.AlarmMilliC.Transform*/TransformAlarmC$0$to_precision_tag;
typedef uint32_t /*HalTimerMilliC.AlarmMilliC.Transform*/TransformAlarmC$0$to_size_type;
typedef T32khz /*HalTimerMilliC.AlarmMilliC.Transform*/TransformAlarmC$0$from_precision_tag;
typedef uint16_t /*HalTimerMilliC.AlarmMilliC.Transform*/TransformAlarmC$0$from_size_type;
typedef /*HalTimerMilliC.AlarmMilliC.Transform*/TransformAlarmC$0$to_precision_tag /*HalTimerMilliC.AlarmMilliC.Transform*/TransformAlarmC$0$Alarm$precision_tag;
typedef /*HalTimerMilliC.AlarmMilliC.Transform*/TransformAlarmC$0$to_size_type /*HalTimerMilliC.AlarmMilliC.Transform*/TransformAlarmC$0$Alarm$size_type;
typedef /*HalTimerMilliC.AlarmMilliC.Transform*/TransformAlarmC$0$from_precision_tag /*HalTimerMilliC.AlarmMilliC.Transform*/TransformAlarmC$0$AlarmFrom$precision_tag;
typedef /*HalTimerMilliC.AlarmMilliC.Transform*/TransformAlarmC$0$from_size_type /*HalTimerMilliC.AlarmMilliC.Transform*/TransformAlarmC$0$AlarmFrom$size_type;
typedef /*HalTimerMilliC.AlarmMilliC.Transform*/TransformAlarmC$0$to_precision_tag /*HalTimerMilliC.AlarmMilliC.Transform*/TransformAlarmC$0$Counter$precision_tag;
typedef /*HalTimerMilliC.AlarmMilliC.Transform*/TransformAlarmC$0$to_size_type /*HalTimerMilliC.AlarmMilliC.Transform*/TransformAlarmC$0$Counter$size_type;
typedef T32khz /*MSP430Counter32khzC.Counter*/MSP430CounterC$0$frequency_tag;
typedef /*MSP430Counter32khzC.Counter*/MSP430CounterC$0$frequency_tag /*MSP430Counter32khzC.Counter*/MSP430CounterC$0$Counter$precision_tag;
typedef uint16_t /*MSP430Counter32khzC.Counter*/MSP430CounterC$0$Counter$size_type;
typedef TMilli /*CounterMilliC.Transform*/TransformCounterC$0$to_precision_tag;
typedef uint32_t /*CounterMilliC.Transform*/TransformCounterC$0$to_size_type;
typedef T32khz /*CounterMilliC.Transform*/TransformCounterC$0$from_precision_tag;
typedef uint16_t /*CounterMilliC.Transform*/TransformCounterC$0$from_size_type;
typedef uint32_t /*CounterMilliC.Transform*/TransformCounterC$0$upper_count_type;
typedef /*CounterMilliC.Transform*/TransformCounterC$0$from_precision_tag /*CounterMilliC.Transform*/TransformCounterC$0$CounterFrom$precision_tag;
typedef /*CounterMilliC.Transform*/TransformCounterC$0$from_size_type /*CounterMilliC.Transform*/TransformCounterC$0$CounterFrom$size_type;
typedef /*CounterMilliC.Transform*/TransformCounterC$0$to_precision_tag /*CounterMilliC.Transform*/TransformCounterC$0$Counter$precision_tag;
typedef /*CounterMilliC.Transform*/TransformCounterC$0$to_size_type /*CounterMilliC.Transform*/TransformCounterC$0$Counter$size_type;
typedef TMilli /*CounterMilliC.CounterToLocalTimeC*/CounterToLocalTimeC$0$precision_tag;
typedef /*CounterMilliC.CounterToLocalTimeC*/CounterToLocalTimeC$0$precision_tag /*CounterMilliC.CounterToLocalTimeC*/CounterToLocalTimeC$0$LocalTime$precision_tag;
typedef /*CounterMilliC.CounterToLocalTimeC*/CounterToLocalTimeC$0$precision_tag /*CounterMilliC.CounterToLocalTimeC*/CounterToLocalTimeC$0$Counter$precision_tag;
typedef uint32_t /*CounterMilliC.CounterToLocalTimeC*/CounterToLocalTimeC$0$Counter$size_type;
typedef TMilli /*HalTimerMilliC.AlarmToTimerC*/AlarmToTimerC$0$precision_tag;
typedef /*HalTimerMilliC.AlarmToTimerC*/AlarmToTimerC$0$precision_tag /*HalTimerMilliC.AlarmToTimerC*/AlarmToTimerC$0$Alarm$precision_tag;
typedef uint32_t /*HalTimerMilliC.AlarmToTimerC*/AlarmToTimerC$0$Alarm$size_type;
typedef /*HalTimerMilliC.AlarmToTimerC*/AlarmToTimerC$0$precision_tag /*HalTimerMilliC.AlarmToTimerC*/AlarmToTimerC$0$Timer$precision_tag;
typedef TMilli /*HalTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC$0$precision_tag;
typedef /*HalTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC$0$precision_tag /*HalTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC$0$TimerFrom$precision_tag;
typedef /*HalTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC$0$precision_tag /*HalTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC$0$Timer$precision_tag;
enum /*ADC8IO14C.TimerMilliC*/TimerMilliC$0$__nesc_unnamed4312 {
  TimerMilliC$0$TIMER_ID = 0U
};
enum /*MainUartPacketC.MainControlC*/MainControlC$1$__nesc_unnamed4313 {
  MainControlC$1$ID = 1U
};
enum /*MainUartPresenceC.MainControlC*/MainControlC$2$__nesc_unnamed4314 {
  MainControlC$2$ID = 2U
};
typedef TMilli TimerWrapC$Timer2$precision_tag;
enum /*MainSPC.MainControlC*/MainControlC$3$__nesc_unnamed4315 {
  MainControlC$3$ID = 3U
};
typedef sp_message_t CC2420AlwaysOnM$PoolEvents$object_type;
typedef sp_message_t CC2420AlwaysOnM$Pool$object_type;
typedef T32khz CC2420AlwaysOnM$AlarmStart$precision_tag;
typedef uint32_t CC2420AlwaysOnM$AlarmStart$size_type;
typedef TMilli CC2420AlwaysOnM$SanityTimer$precision_tag;
typedef T32khz CC2420AlwaysOnM$AlarmStop$precision_tag;
typedef uint32_t CC2420AlwaysOnM$AlarmStop$size_type;
typedef T32khz CC2420AlwaysOnM$TimeStamping$precision_tag;
typedef uint32_t CC2420AlwaysOnM$TimeStamping$size_type;
typedef T32khz CC2420RadioM$BackoffAlarm32khz$precision_tag;
typedef uint16_t CC2420RadioM$BackoffAlarm32khz$size_type;
typedef T32khz CC2420RadioM$RadioActiveTime$precision_tag;
typedef uint32_t CC2420RadioM$RadioActiveTime$size_type;
typedef T32khz CC2420RadioM$Counter32khz$precision_tag;
typedef uint32_t CC2420RadioM$Counter32khz$size_type;
typedef T32khz CC2420RadioM$Counter32khz16$precision_tag;
typedef uint16_t CC2420RadioM$Counter32khz16$size_type;
enum /*CC2420RadioC.BackoffAlarm32khzC.MSP430Timer*/MSP430Timer32khzC$2$__nesc_unnamed4316 {
  MSP430Timer32khzC$2$ALARM_ID = 2U
};
typedef T32khz /*CC2420RadioC.BackoffAlarm32khzC.MSP430Alarm*/MSP430AlarmC$1$frequency_tag;
typedef /*CC2420RadioC.BackoffAlarm32khzC.MSP430Alarm*/MSP430AlarmC$1$frequency_tag /*CC2420RadioC.BackoffAlarm32khzC.MSP430Alarm*/MSP430AlarmC$1$Alarm$precision_tag;
typedef uint16_t /*CC2420RadioC.BackoffAlarm32khzC.MSP430Alarm*/MSP430AlarmC$1$Alarm$size_type;
typedef T32khz /*Counter32khzC.Transform*/TransformCounterC$1$to_precision_tag;
typedef uint32_t /*Counter32khzC.Transform*/TransformCounterC$1$to_size_type;
typedef T32khz /*Counter32khzC.Transform*/TransformCounterC$1$from_precision_tag;
typedef uint16_t /*Counter32khzC.Transform*/TransformCounterC$1$from_size_type;
typedef uint16_t /*Counter32khzC.Transform*/TransformCounterC$1$upper_count_type;
typedef /*Counter32khzC.Transform*/TransformCounterC$1$from_precision_tag /*Counter32khzC.Transform*/TransformCounterC$1$CounterFrom$precision_tag;
typedef /*Counter32khzC.Transform*/TransformCounterC$1$from_size_type /*Counter32khzC.Transform*/TransformCounterC$1$CounterFrom$size_type;
typedef /*Counter32khzC.Transform*/TransformCounterC$1$to_precision_tag /*Counter32khzC.Transform*/TransformCounterC$1$Counter$precision_tag;
typedef /*Counter32khzC.Transform*/TransformCounterC$1$to_size_type /*Counter32khzC.Transform*/TransformCounterC$1$Counter$size_type;
typedef T32khz /*Counter32khzC.CounterToLocalTimeC*/CounterToLocalTimeC$1$precision_tag;
typedef /*Counter32khzC.CounterToLocalTimeC*/CounterToLocalTimeC$1$precision_tag /*Counter32khzC.CounterToLocalTimeC*/CounterToLocalTimeC$1$LocalTime$precision_tag;
typedef /*Counter32khzC.CounterToLocalTimeC*/CounterToLocalTimeC$1$precision_tag /*Counter32khzC.CounterToLocalTimeC*/CounterToLocalTimeC$1$Counter$precision_tag;
typedef uint32_t /*Counter32khzC.CounterToLocalTimeC*/CounterToLocalTimeC$1$Counter$size_type;
enum /*CC2420RadioC.CmdCCAFiredC.ResourceC.ResourceUSARTP*/MSP430ResourceUSART0P$0$__nesc_unnamed4317 {
  MSP430ResourceUSART0P$0$ID = 0U + 1
};
enum /*CC2420RadioC.CmdSplitControlInitC.ResourceC.ResourceUSARTP*/MSP430ResourceUSART0P$1$__nesc_unnamed4318 {
  MSP430ResourceUSART0P$1$ID = 1U + 1
};
enum /*CC2420RadioC.CmdSplitControlStartC.ResourceC.ResourceUSARTP*/MSP430ResourceUSART0P$2$__nesc_unnamed4319 {
  MSP430ResourceUSART0P$2$ID = 2U + 1
};
enum /*CC2420RadioC.CmdSplitControlStopC.ResourceC.ResourceUSARTP*/MSP430ResourceUSART0P$3$__nesc_unnamed4320 {
  MSP430ResourceUSART0P$3$ID = 3U + 1
};
enum /*CC2420RadioC.CmdCmds.ResourceC.ResourceUSARTP*/MSP430ResourceUSART0P$4$__nesc_unnamed4321 {
  MSP430ResourceUSART0P$4$ID = 4U + 1
};
enum /*CC2420RadioC.CmdFlushRXFIFOC.ResourceC.ResourceUSARTP*/MSP430ResourceUSART0P$5$__nesc_unnamed4322 {
  MSP430ResourceUSART0P$5$ID = 5U + 1
};
enum /*CC2420RadioC.CmdReceiveC.ResourceC.ResourceUSARTP*/MSP430ResourceUSART0P$6$__nesc_unnamed4323 {
  MSP430ResourceUSART0P$6$ID = 6U + 1
};
enum /*CC2420RadioC.CmdTransmitC.ResourceC.ResourceUSARTP*/MSP430ResourceUSART0P$7$__nesc_unnamed4324 {
  MSP430ResourceUSART0P$7$ID = 7U + 1
};
enum /*CC2420RadioC.CmdTryToSendC.ResourceC.ResourceUSARTP*/MSP430ResourceUSART0P$8$__nesc_unnamed4325 {
  MSP430ResourceUSART0P$8$ID = 8U + 1
};
typedef T32khz CC2420TimeStampingM$LocalTime$precision_tag;
typedef T32khz CC2420TimeStampingM$TimeStamping$precision_tag;
typedef uint32_t CC2420TimeStampingM$TimeStamping$size_type;
enum /*CC2420TimeStampingC.CmdWriteTimeStampC.ResourceC.ResourceUSARTP*/MSP430ResourceUSART0P$9$__nesc_unnamed4326 {
  MSP430ResourceUSART0P$9$ID = 9U + 1
};
typedef T32khz /*CC2420SyncAlwaysOnC.DualAlarmC*/VirtualizeAlarmC$0$precision_tag;
typedef uint32_t /*CC2420SyncAlwaysOnC.DualAlarmC*/VirtualizeAlarmC$0$size_type;
typedef /*CC2420SyncAlwaysOnC.DualAlarmC*/VirtualizeAlarmC$0$precision_tag /*CC2420SyncAlwaysOnC.DualAlarmC*/VirtualizeAlarmC$0$Alarm$precision_tag;
typedef /*CC2420SyncAlwaysOnC.DualAlarmC*/VirtualizeAlarmC$0$size_type /*CC2420SyncAlwaysOnC.DualAlarmC*/VirtualizeAlarmC$0$Alarm$size_type;
typedef /*CC2420SyncAlwaysOnC.DualAlarmC*/VirtualizeAlarmC$0$precision_tag /*CC2420SyncAlwaysOnC.DualAlarmC*/VirtualizeAlarmC$0$AlarmFrom$precision_tag;
typedef /*CC2420SyncAlwaysOnC.DualAlarmC*/VirtualizeAlarmC$0$size_type /*CC2420SyncAlwaysOnC.DualAlarmC*/VirtualizeAlarmC$0$AlarmFrom$size_type;
enum /*CC2420SyncAlwaysOnC.AlarmC.MSP430Timer*/MSP430Timer32khzC$3$__nesc_unnamed4327 {
  MSP430Timer32khzC$3$ALARM_ID = 3U
};
typedef T32khz /*CC2420SyncAlwaysOnC.AlarmC.MSP430Alarm*/MSP430AlarmC$2$frequency_tag;
typedef /*CC2420SyncAlwaysOnC.AlarmC.MSP430Alarm*/MSP430AlarmC$2$frequency_tag /*CC2420SyncAlwaysOnC.AlarmC.MSP430Alarm*/MSP430AlarmC$2$Alarm$precision_tag;
typedef uint16_t /*CC2420SyncAlwaysOnC.AlarmC.MSP430Alarm*/MSP430AlarmC$2$Alarm$size_type;
typedef T32khz /*CC2420SyncAlwaysOnC.AlarmC.Transform*/TransformAlarmC$1$to_precision_tag;
typedef uint32_t /*CC2420SyncAlwaysOnC.AlarmC.Transform*/TransformAlarmC$1$to_size_type;
typedef T32khz /*CC2420SyncAlwaysOnC.AlarmC.Transform*/TransformAlarmC$1$from_precision_tag;
typedef uint16_t /*CC2420SyncAlwaysOnC.AlarmC.Transform*/TransformAlarmC$1$from_size_type;
typedef /*CC2420SyncAlwaysOnC.AlarmC.Transform*/TransformAlarmC$1$to_precision_tag /*CC2420SyncAlwaysOnC.AlarmC.Transform*/TransformAlarmC$1$Alarm$precision_tag;
typedef /*CC2420SyncAlwaysOnC.AlarmC.Transform*/TransformAlarmC$1$to_size_type /*CC2420SyncAlwaysOnC.AlarmC.Transform*/TransformAlarmC$1$Alarm$size_type;
typedef /*CC2420SyncAlwaysOnC.AlarmC.Transform*/TransformAlarmC$1$from_precision_tag /*CC2420SyncAlwaysOnC.AlarmC.Transform*/TransformAlarmC$1$AlarmFrom$precision_tag;
typedef /*CC2420SyncAlwaysOnC.AlarmC.Transform*/TransformAlarmC$1$from_size_type /*CC2420SyncAlwaysOnC.AlarmC.Transform*/TransformAlarmC$1$AlarmFrom$size_type;
typedef /*CC2420SyncAlwaysOnC.AlarmC.Transform*/TransformAlarmC$1$to_precision_tag /*CC2420SyncAlwaysOnC.AlarmC.Transform*/TransformAlarmC$1$Counter$precision_tag;
typedef /*CC2420SyncAlwaysOnC.AlarmC.Transform*/TransformAlarmC$1$to_size_type /*CC2420SyncAlwaysOnC.AlarmC.Transform*/TransformAlarmC$1$Counter$size_type;
enum /*CC2420SyncAlwaysOnC.TimerMilliC*/TimerMilliC$1$__nesc_unnamed4328 {
  TimerMilliC$1$TIMER_ID = 2U
};
typedef sp_message_t /*SPC.MessagePool*/ObjectPoolC$0$object_type;
typedef /*SPC.MessagePool*/ObjectPoolC$0$object_type /*SPC.MessagePool*/ObjectPoolC$0$PoolEvents$object_type;
typedef /*SPC.MessagePool*/ObjectPoolC$0$object_type /*SPC.MessagePool*/ObjectPoolC$0$Pool$object_type;
typedef sp_neighbor_t /*SPC.NeighborTable*/ObjectPoolC$1$object_type;
typedef /*SPC.NeighborTable*/ObjectPoolC$1$object_type /*SPC.NeighborTable*/ObjectPoolC$1$PoolEvents$object_type;
typedef /*SPC.NeighborTable*/ObjectPoolC$1$object_type /*SPC.NeighborTable*/ObjectPoolC$1$Pool$object_type;
typedef sp_message_t SPM$PoolEvents$object_type;
typedef sp_message_t SPM$Pool$object_type;
typedef T32khz SPM$LocalTime$precision_tag;
typedef sp_message_t SPDataM$Pool$object_type;
typedef T32khz SPDataM$LocalTime$precision_tag;
typedef sp_neighbor_t SPNeighborTableM$NeighborTableEvents$object_type;
typedef sp_neighbor_t SPNeighborTableM$NeighborTable$object_type;
typedef T32khz SPUtilM$TimeStamping$precision_tag;
typedef uint32_t SPUtilM$TimeStamping$size_type;
enum /*MSP430ADC12C.ResourceC*/MSP430ResourceTimerAC$1$__nesc_unnamed4329 {
  MSP430ResourceTimerAC$1$ID = 1U + 1
};
typedef TMilli RefVoltM$SwitchOffTimer$precision_tag;
typedef TMilli RefVoltM$SwitchOnTimer$precision_tag;
enum /*RefVoltC.Timer1*/TimerMilliC$2$__nesc_unnamed4330 {
  TimerMilliC$2$TIMER_ID = 3U
};
enum /*RefVoltC.Timer2*/TimerMilliC$3$__nesc_unnamed4331 {
  TimerMilliC$3$TIMER_ID = 4U
};
# 46 "/Users/jingyuancheng/tinyos/moteiv/tos/lib/sched/Init.nc"
static result_t PlatformP$HPLInit$init(void );
# 30 "/Users/jingyuancheng/tinyos/moteiv/tinyos-1.x/tos/platform/msp430/HPLInitM.nc"
static result_t HPLInitM$init(void );
# 29 "/Users/jingyuancheng/tinyos/moteiv/tos/platform/msp430/timer/MSP430ClockInit.nc"
static void MSP430ClockM$MSP430ClockInit$default$initTimerB(void );


static void MSP430ClockM$MSP430ClockInit$defaultInitTimerA(void );
#line 28
static void MSP430ClockM$MSP430ClockInit$default$initTimerA(void );




static void MSP430ClockM$MSP430ClockInit$defaultInitTimerB(void );
#line 27
static void MSP430ClockM$MSP430ClockInit$default$initClocks(void );



static void MSP430ClockM$MSP430ClockInit$defaultInitClocks(void );
# 63 "/Users/jingyuancheng/tinyos/moteiv/tinyos-1.x/tos/interfaces/StdControl.nc"
static result_t MSP430ClockM$Init$init(void );






static result_t MSP430ClockM$Init$start(void );
# 33 "/Users/jingyuancheng/tinyos/moteiv/tos/platform/msp430/timer/MSP430Timer.nc"
static void MSP430DCOCalibP$TimerA$overflow(void );
# 34 "/Users/jingyuancheng/tinyos/moteiv/tos/platform/msp430/timer/MSP430Compare.nc"
static void MSP430DCOCalibP$TimerCompareB$fired(void );
# 70 "/Users/jingyuancheng/tinyos/moteiv/tos/lib/resource/ResourceCmd.nc"
static void MSP430DCOCalibP$ResourceTimerA$granted(uint8_t rh);
# 33 "/Users/jingyuancheng/tinyos/moteiv/tos/platform/msp430/timer/MSP430Timer.nc"
static void MSP430DCOCalibP$TimerB$overflow(void );
# 75 "/Users/jingyuancheng/tinyos/moteiv/tos/lib/resource/ResourceCmd.nc"
static void /*MSP430ArbiterTimerAC.ArbiterC.ArbiterP*/FcfsArbiterP$0$ResourceCmd$release(
# 22 "/Users/jingyuancheng/tinyos/moteiv/tos/lib/resource/FcfsArbiterP.nc"
uint8_t arg_0x101b3b660);
# 63 "/Users/jingyuancheng/tinyos/moteiv/tos/lib/resource/ResourceCmd.nc"
static void /*MSP430ArbiterTimerAC.ArbiterC.ArbiterP*/FcfsArbiterP$0$ResourceCmd$deferRequest(
# 22 "/Users/jingyuancheng/tinyos/moteiv/tos/lib/resource/FcfsArbiterP.nc"
uint8_t arg_0x101b3b660);
# 70 "/Users/jingyuancheng/tinyos/moteiv/tos/lib/resource/ResourceCmd.nc"
static void /*MSP430ArbiterTimerAC.ArbiterC.ArbiterP*/FcfsArbiterP$0$ResourceCmd$default$granted(
# 22 "/Users/jingyuancheng/tinyos/moteiv/tos/lib/resource/FcfsArbiterP.nc"
uint8_t arg_0x101b3b660, 
# 70 "/Users/jingyuancheng/tinyos/moteiv/tos/lib/resource/ResourceCmd.nc"
uint8_t rh);
# 28 "/Users/jingyuancheng/tinyos/moteiv/tos/lib/resource/ResourceValidate.nc"
static bool /*MSP430ArbiterTimerAC.ArbiterC.ArbiterP*/FcfsArbiterP$0$ResourceValidate$validateUser(uint8_t rh);
# 85 "/Users/jingyuancheng/tinyos/moteiv/tos/lib/resource/ResourceCmdAsync.nc"
static void /*MSP430ArbiterTimerAC.ArbiterC.ArbiterP*/FcfsArbiterP$0$ResourceCmdAsync$release(
# 23 "/Users/jingyuancheng/tinyos/moteiv/tos/lib/resource/FcfsArbiterP.nc"
uint8_t arg_0x101b3a698);
# 73 "/Users/jingyuancheng/tinyos/moteiv/tos/lib/resource/ResourceCmdAsync.nc"
static uint8_t /*MSP430ArbiterTimerAC.ArbiterC.ArbiterP*/FcfsArbiterP$0$ResourceCmdAsync$immediateRequest(
# 23 "/Users/jingyuancheng/tinyos/moteiv/tos/lib/resource/FcfsArbiterP.nc"
uint8_t arg_0x101b3a698, 
# 73 "/Users/jingyuancheng/tinyos/moteiv/tos/lib/resource/ResourceCmdAsync.nc"
uint8_t rh);






static void /*MSP430ArbiterTimerAC.ArbiterC.ArbiterP*/FcfsArbiterP$0$ResourceCmdAsync$default$granted(
# 23 "/Users/jingyuancheng/tinyos/moteiv/tos/lib/resource/FcfsArbiterP.nc"
uint8_t arg_0x101b3a698, 
# 80 "/Users/jingyuancheng/tinyos/moteiv/tos/lib/resource/ResourceCmdAsync.nc"
uint8_t rh);
# 46 "/Users/jingyuancheng/tinyos/moteiv/tos/lib/sched/Init.nc"
static result_t /*MSP430ArbiterTimerAC.ArbiterC.ArbiterP*/FcfsArbiterP$0$Init$init(void );
# 58 "/Users/jingyuancheng/tinyos/moteiv/tos/lib/sched/TaskBasic.nc"
static void /*MSP430ArbiterTimerAC.ArbiterC.ArbiterP*/FcfsArbiterP$0$GrantTask$runTask(void );
# 78 "/Users/jingyuancheng/tinyos/moteiv/tos/lib/resource/Resource.nc"
static void /*MSP430ArbiterTimerAC.ArbiterC.ArbiterP*/FcfsArbiterP$0$Resource$release(
# 21 "/Users/jingyuancheng/tinyos/moteiv/tos/lib/resource/FcfsArbiterP.nc"
uint8_t arg_0x101b3c648);
# 58 "/Users/jingyuancheng/tinyos/moteiv/tos/lib/resource/Resource.nc"
static void /*MSP430ArbiterTimerAC.ArbiterC.ArbiterP*/FcfsArbiterP$0$Resource$request(
# 21 "/Users/jingyuancheng/tinyos/moteiv/tos/lib/resource/FcfsArbiterP.nc"
uint8_t arg_0x101b3c648);
# 73 "/Users/jingyuancheng/tinyos/moteiv/tos/lib/resource/Resource.nc"
static void /*MSP430ArbiterTimerAC.ArbiterC.ArbiterP*/FcfsArbiterP$0$Resource$default$granted(
# 21 "/Users/jingyuancheng/tinyos/moteiv/tos/lib/resource/FcfsArbiterP.nc"
uint8_t arg_0x101b3c648);
# 50 "/Users/jingyuancheng/tinyos/moteiv/tos/lib/sched/TaskBasic.nc"
static result_t SchedulerBasicP$TaskBasic$postUrgentTask(
# 51 "/Users/jingyuancheng/tinyos/moteiv/tos/lib/sched/SchedulerBasicP.nc"
uint8_t arg_0x101550368);
# 49 "/Users/jingyuancheng/tinyos/moteiv/tos/lib/sched/TaskBasic.nc"
static result_t SchedulerBasicP$TaskBasic$postTask(
# 51 "/Users/jingyuancheng/tinyos/moteiv/tos/lib/sched/SchedulerBasicP.nc"
uint8_t arg_0x101550368);
# 58 "/Users/jingyuancheng/tinyos/moteiv/tos/lib/sched/TaskBasic.nc"
static void SchedulerBasicP$TaskBasic$default$runTask(
# 51 "/Users/jingyuancheng/tinyos/moteiv/tos/lib/sched/SchedulerBasicP.nc"
uint8_t arg_0x101550368);
# 41 "/Users/jingyuancheng/tinyos/moteiv/tos/lib/sched/Scheduler.nc"
static void SchedulerBasicP$Scheduler$init(void );








static bool SchedulerBasicP$Scheduler$runNextTask(bool l_sleep);
# 29 "/Users/jingyuancheng/tinyos/moteiv/tos/lib/resource/ResourceConfigure.nc"
static void MSP430ResourceConfigTimerAP$WrapConfigTimerA$configure(
# 16 "/Users/jingyuancheng/tinyos/moteiv/tos/platform/msp430/resource/MSP430ResourceConfigTimerAP.nc"
uint8_t arg_0x101bbd060);
# 29 "/Users/jingyuancheng/tinyos/moteiv/tos/lib/resource/ResourceConfigure.nc"
static void MSP430ResourceConfigTimerAP$ConfigTimerA$default$configure(
# 17 "/Users/jingyuancheng/tinyos/moteiv/tos/platform/msp430/resource/MSP430ResourceConfigTimerAP.nc"
uint8_t arg_0x101bbdc98);
# 33 "/Users/jingyuancheng/tinyos/moteiv/tos/platform/msp430/timer/MSP430Timer.nc"
static void MSP430ResourceConfigTimerAP$TimerA$overflow(void );
# 54 "/Users/jingyuancheng/tinyos/moteiv/tos/lib/resource/Arbiter.nc"
static void MSP430ResourceConfigTimerAP$Arbiter$requested(void );





static void MSP430ResourceConfigTimerAP$Arbiter$idle(void );
# 4 "/Users/jingyuancheng/tinyos/moteiv/tos/platform/msp430/timer/MSP430TimerEvent.nc"
static void /*MSP430TimerC.MSP430TimerA*/MSP430TimerM$0$VectorTimerX0$fired(void );
#line 4
static void /*MSP430TimerC.MSP430TimerA*/MSP430TimerM$0$Overflow$fired(void );
#line 4
static void /*MSP430TimerC.MSP430TimerA*/MSP430TimerM$0$VectorTimerX1$fired(void );
#line 4
static void /*MSP430TimerC.MSP430TimerA*/MSP430TimerM$0$Event$default$fired(
# 39 "/Users/jingyuancheng/tinyos/moteiv/tos/platform/msp430/timer/MSP430TimerM.nc"
uint8_t arg_0x101bf0748);
# 37 "/Users/jingyuancheng/tinyos/moteiv/tos/platform/msp430/timer/MSP430Timer.nc"
static void /*MSP430TimerC.MSP430TimerA*/MSP430TimerM$0$Timer$clear(void );


static void /*MSP430TimerC.MSP430TimerA*/MSP430TimerM$0$Timer$setClockSource(uint16_t clockSource);
#line 30
static uint16_t /*MSP430TimerC.MSP430TimerA*/MSP430TimerM$0$Timer$get(void );








static void /*MSP430TimerC.MSP430TimerA*/MSP430TimerM$0$Timer$disableEvents(void );
#line 35
static void /*MSP430TimerC.MSP430TimerA*/MSP430TimerM$0$Timer$setMode(int mode);





static void /*MSP430TimerC.MSP430TimerA*/MSP430TimerM$0$Timer$setInputDivider(uint16_t inputDivider);
# 4 "/Users/jingyuancheng/tinyos/moteiv/tos/platform/msp430/timer/MSP430TimerEvent.nc"
static void /*MSP430TimerC.MSP430TimerB*/MSP430TimerM$1$VectorTimerX0$fired(void );
#line 4
static void /*MSP430TimerC.MSP430TimerB*/MSP430TimerM$1$Overflow$fired(void );
#line 4
static void /*MSP430TimerC.MSP430TimerB*/MSP430TimerM$1$VectorTimerX1$fired(void );
#line 4
static void /*MSP430TimerC.MSP430TimerB*/MSP430TimerM$1$Event$default$fired(
# 39 "/Users/jingyuancheng/tinyos/moteiv/tos/platform/msp430/timer/MSP430TimerM.nc"
uint8_t arg_0x101bf0748);
# 30 "/Users/jingyuancheng/tinyos/moteiv/tos/platform/msp430/timer/MSP430Timer.nc"
static uint16_t /*MSP430TimerC.MSP430TimerB*/MSP430TimerM$1$Timer$get(void );
static bool /*MSP430TimerC.MSP430TimerB*/MSP430TimerM$1$Timer$isOverflowPending(void );
# 32 "/Users/jingyuancheng/tinyos/moteiv/tos/platform/msp430/timer/MSP430Capture.nc"
static uint16_t /*MSP430TimerC.MSP430TimerA0*/MSP430TimerCapComM$0$Capture$getEvent(void );
#line 74
static void /*MSP430TimerC.MSP430TimerA0*/MSP430TimerCapComM$0$Capture$default$captured(uint16_t time);
# 30 "/Users/jingyuancheng/tinyos/moteiv/tos/platform/msp430/timer/MSP430TimerControl.nc"
static MSP430CompareControl_t /*MSP430TimerC.MSP430TimerA0*/MSP430TimerCapComM$0$Control$getControl(void );



static void /*MSP430TimerC.MSP430TimerA0*/MSP430TimerCapComM$0$Control$setControl(MSP430CompareControl_t control);
# 4 "/Users/jingyuancheng/tinyos/moteiv/tos/platform/msp430/timer/MSP430TimerEvent.nc"
static void /*MSP430TimerC.MSP430TimerA0*/MSP430TimerCapComM$0$Event$fired(void );
# 30 "/Users/jingyuancheng/tinyos/moteiv/tos/platform/msp430/timer/MSP430Compare.nc"
static void /*MSP430TimerC.MSP430TimerA0*/MSP430TimerCapComM$0$Compare$setEvent(uint16_t time);
# 33 "/Users/jingyuancheng/tinyos/moteiv/tos/platform/msp430/timer/MSP430Timer.nc"
static void /*MSP430TimerC.MSP430TimerA0*/MSP430TimerCapComM$0$Timer$overflow(void );
# 32 "/Users/jingyuancheng/tinyos/moteiv/tos/platform/msp430/timer/MSP430Capture.nc"
static uint16_t /*MSP430TimerC.MSP430TimerA1*/MSP430TimerCapComM$1$Capture$getEvent(void );
#line 74
static void /*MSP430TimerC.MSP430TimerA1*/MSP430TimerCapComM$1$Capture$default$captured(uint16_t time);
# 30 "/Users/jingyuancheng/tinyos/moteiv/tos/platform/msp430/timer/MSP430TimerControl.nc"
static MSP430CompareControl_t /*MSP430TimerC.MSP430TimerA1*/MSP430TimerCapComM$1$Control$getControl(void );



static void /*MSP430TimerC.MSP430TimerA1*/MSP430TimerCapComM$1$Control$setControl(MSP430CompareControl_t control);
# 4 "/Users/jingyuancheng/tinyos/moteiv/tos/platform/msp430/timer/MSP430TimerEvent.nc"
static void /*MSP430TimerC.MSP430TimerA1*/MSP430TimerCapComM$1$Event$fired(void );
# 30 "/Users/jingyuancheng/tinyos/moteiv/tos/platform/msp430/timer/MSP430Compare.nc"
static void /*MSP430TimerC.MSP430TimerA1*/MSP430TimerCapComM$1$Compare$setEvent(uint16_t time);
# 33 "/Users/jingyuancheng/tinyos/moteiv/tos/platform/msp430/timer/MSP430Timer.nc"
static void /*MSP430TimerC.MSP430TimerA1*/MSP430TimerCapComM$1$Timer$overflow(void );
# 32 "/Users/jingyuancheng/tinyos/moteiv/tos/platform/msp430/timer/MSP430Capture.nc"
static uint16_t /*MSP430TimerC.MSP430TimerA2*/MSP430TimerCapComM$2$Capture$getEvent(void );
#line 74
static void /*MSP430TimerC.MSP430TimerA2*/MSP430TimerCapComM$2$Capture$default$captured(uint16_t time);
# 30 "/Users/jingyuancheng/tinyos/moteiv/tos/platform/msp430/timer/MSP430TimerControl.nc"
static MSP430CompareControl_t /*MSP430TimerC.MSP430TimerA2*/MSP430TimerCapComM$2$Control$getControl(void );
# 4 "/Users/jingyuancheng/tinyos/moteiv/tos/platform/msp430/timer/MSP430TimerEvent.nc"
static void /*MSP430TimerC.MSP430TimerA2*/MSP430TimerCapComM$2$Event$fired(void );
# 34 "/Users/jingyuancheng/tinyos/moteiv/tos/platform/msp430/timer/MSP430Compare.nc"
static void /*MSP430TimerC.MSP430TimerA2*/MSP430TimerCapComM$2$Compare$default$fired(void );
# 33 "/Users/jingyuancheng/tinyos/moteiv/tos/platform/msp430/timer/MSP430Timer.nc"
static void /*MSP430TimerC.MSP430TimerA2*/MSP430TimerCapComM$2$Timer$overflow(void );
# 32 "/Users/jingyuancheng/tinyos/moteiv/tos/platform/msp430/timer/MSP430Capture.nc"
static uint16_t /*MSP430TimerC.MSP430TimerB0*/MSP430TimerCapComM$3$Capture$getEvent(void );
#line 74
static void /*MSP430TimerC.MSP430TimerB0*/MSP430TimerCapComM$3$Capture$default$captured(uint16_t time);
# 30 "/Users/jingyuancheng/tinyos/moteiv/tos/platform/msp430/timer/MSP430TimerControl.nc"
static MSP430CompareControl_t /*MSP430TimerC.MSP430TimerB0*/MSP430TimerCapComM$3$Control$getControl(void );








static void /*MSP430TimerC.MSP430TimerB0*/MSP430TimerCapComM$3$Control$disableEvents(void );
#line 31
static bool /*MSP430TimerC.MSP430TimerB0*/MSP430TimerCapComM$3$Control$isInterruptPending(void );
static void /*MSP430TimerC.MSP430TimerB0*/MSP430TimerCapComM$3$Control$clearPendingInterrupt(void );
# 4 "/Users/jingyuancheng/tinyos/moteiv/tos/platform/msp430/timer/MSP430TimerEvent.nc"
static void /*MSP430TimerC.MSP430TimerB0*/MSP430TimerCapComM$3$Event$fired(void );
# 31 "/Users/jingyuancheng/tinyos/moteiv/tos/platform/msp430/timer/MSP430Compare.nc"
static void /*MSP430TimerC.MSP430TimerB0*/MSP430TimerCapComM$3$Compare$setEventFromPrev(uint16_t delta);
static void /*MSP430TimerC.MSP430TimerB0*/MSP430TimerCapComM$3$Compare$setEventFromNow(uint16_t delta);
# 33 "/Users/jingyuancheng/tinyos/moteiv/tos/platform/msp430/timer/MSP430Timer.nc"
static void /*MSP430TimerC.MSP430TimerB0*/MSP430TimerCapComM$3$Timer$overflow(void );
# 32 "/Users/jingyuancheng/tinyos/moteiv/tos/platform/msp430/timer/MSP430Capture.nc"
static uint16_t /*MSP430TimerC.MSP430TimerB1*/MSP430TimerCapComM$4$Capture$getEvent(void );
#line 56
static void /*MSP430TimerC.MSP430TimerB1*/MSP430TimerCapComM$4$Capture$clearOverflow(void );
#line 51
static bool /*MSP430TimerC.MSP430TimerB1*/MSP430TimerCapComM$4$Capture$isOverflowPending(void );
# 36 "/Users/jingyuancheng/tinyos/moteiv/tos/platform/msp430/timer/MSP430TimerControl.nc"
static void /*MSP430TimerC.MSP430TimerB1*/MSP430TimerCapComM$4$Control$setControlAsCapture(uint8_t capture_mode);
#line 30
static MSP430CompareControl_t /*MSP430TimerC.MSP430TimerB1*/MSP430TimerCapComM$4$Control$getControl(void );







static void /*MSP430TimerC.MSP430TimerB1*/MSP430TimerCapComM$4$Control$enableEvents(void );
static void /*MSP430TimerC.MSP430TimerB1*/MSP430TimerCapComM$4$Control$disableEvents(void );
#line 32
static void /*MSP430TimerC.MSP430TimerB1*/MSP430TimerCapComM$4$Control$clearPendingInterrupt(void );
# 4 "/Users/jingyuancheng/tinyos/moteiv/tos/platform/msp430/timer/MSP430TimerEvent.nc"
static void /*MSP430TimerC.MSP430TimerB1*/MSP430TimerCapComM$4$Event$fired(void );
# 34 "/Users/jingyuancheng/tinyos/moteiv/tos/platform/msp430/timer/MSP430Compare.nc"
static void /*MSP430TimerC.MSP430TimerB1*/MSP430TimerCapComM$4$Compare$default$fired(void );
# 33 "/Users/jingyuancheng/tinyos/moteiv/tos/platform/msp430/timer/MSP430Timer.nc"
static void /*MSP430TimerC.MSP430TimerB1*/MSP430TimerCapComM$4$Timer$overflow(void );
# 32 "/Users/jingyuancheng/tinyos/moteiv/tos/platform/msp430/timer/MSP430Capture.nc"
static uint16_t /*MSP430TimerC.MSP430TimerB2*/MSP430TimerCapComM$5$Capture$getEvent(void );
#line 74
static void /*MSP430TimerC.MSP430TimerB2*/MSP430TimerCapComM$5$Capture$default$captured(uint16_t time);
# 30 "/Users/jingyuancheng/tinyos/moteiv/tos/platform/msp430/timer/MSP430TimerControl.nc"
static MSP430CompareControl_t /*MSP430TimerC.MSP430TimerB2*/MSP430TimerCapComM$5$Control$getControl(void );







static void /*MSP430TimerC.MSP430TimerB2*/MSP430TimerCapComM$5$Control$enableEvents(void );
#line 35
static void /*MSP430TimerC.MSP430TimerB2*/MSP430TimerCapComM$5$Control$setControlAsCompare(void );



static void /*MSP430TimerC.MSP430TimerB2*/MSP430TimerCapComM$5$Control$disableEvents(void );
#line 32
static void /*MSP430TimerC.MSP430TimerB2*/MSP430TimerCapComM$5$Control$clearPendingInterrupt(void );
# 4 "/Users/jingyuancheng/tinyos/moteiv/tos/platform/msp430/timer/MSP430TimerEvent.nc"
static void /*MSP430TimerC.MSP430TimerB2*/MSP430TimerCapComM$5$Event$fired(void );
# 30 "/Users/jingyuancheng/tinyos/moteiv/tos/platform/msp430/timer/MSP430Compare.nc"
static void /*MSP430TimerC.MSP430TimerB2*/MSP430TimerCapComM$5$Compare$setEvent(uint16_t time);

static void /*MSP430TimerC.MSP430TimerB2*/MSP430TimerCapComM$5$Compare$setEventFromNow(uint16_t delta);
# 33 "/Users/jingyuancheng/tinyos/moteiv/tos/platform/msp430/timer/MSP430Timer.nc"
static void /*MSP430TimerC.MSP430TimerB2*/MSP430TimerCapComM$5$Timer$overflow(void );
# 32 "/Users/jingyuancheng/tinyos/moteiv/tos/platform/msp430/timer/MSP430Capture.nc"
static uint16_t /*MSP430TimerC.MSP430TimerB3*/MSP430TimerCapComM$6$Capture$getEvent(void );
#line 74
static void /*MSP430TimerC.MSP430TimerB3*/MSP430TimerCapComM$6$Capture$default$captured(uint16_t time);
# 30 "/Users/jingyuancheng/tinyos/moteiv/tos/platform/msp430/timer/MSP430TimerControl.nc"
static MSP430CompareControl_t /*MSP430TimerC.MSP430TimerB3*/MSP430TimerCapComM$6$Control$getControl(void );







static void /*MSP430TimerC.MSP430TimerB3*/MSP430TimerCapComM$6$Control$enableEvents(void );

static bool /*MSP430TimerC.MSP430TimerB3*/MSP430TimerCapComM$6$Control$areEventsEnabled(void );
#line 35
static void /*MSP430TimerC.MSP430TimerB3*/MSP430TimerCapComM$6$Control$setControlAsCompare(void );



static void /*MSP430TimerC.MSP430TimerB3*/MSP430TimerCapComM$6$Control$disableEvents(void );
#line 32
static void /*MSP430TimerC.MSP430TimerB3*/MSP430TimerCapComM$6$Control$clearPendingInterrupt(void );
# 4 "/Users/jingyuancheng/tinyos/moteiv/tos/platform/msp430/timer/MSP430TimerEvent.nc"
static void /*MSP430TimerC.MSP430TimerB3*/MSP430TimerCapComM$6$Event$fired(void );
# 30 "/Users/jingyuancheng/tinyos/moteiv/tos/platform/msp430/timer/MSP430Compare.nc"
static void /*MSP430TimerC.MSP430TimerB3*/MSP430TimerCapComM$6$Compare$setEvent(uint16_t time);

static void /*MSP430TimerC.MSP430TimerB3*/MSP430TimerCapComM$6$Compare$setEventFromNow(uint16_t delta);
# 33 "/Users/jingyuancheng/tinyos/moteiv/tos/platform/msp430/timer/MSP430Timer.nc"
static void /*MSP430TimerC.MSP430TimerB3*/MSP430TimerCapComM$6$Timer$overflow(void );
# 32 "/Users/jingyuancheng/tinyos/moteiv/tos/platform/msp430/timer/MSP430Capture.nc"
static uint16_t /*MSP430TimerC.MSP430TimerB4*/MSP430TimerCapComM$7$Capture$getEvent(void );
#line 74
static void /*MSP430TimerC.MSP430TimerB4*/MSP430TimerCapComM$7$Capture$default$captured(uint16_t time);
# 30 "/Users/jingyuancheng/tinyos/moteiv/tos/platform/msp430/timer/MSP430TimerControl.nc"
static MSP430CompareControl_t /*MSP430TimerC.MSP430TimerB4*/MSP430TimerCapComM$7$Control$getControl(void );







static void /*MSP430TimerC.MSP430TimerB4*/MSP430TimerCapComM$7$Control$enableEvents(void );
#line 35
static void /*MSP430TimerC.MSP430TimerB4*/MSP430TimerCapComM$7$Control$setControlAsCompare(void );



static void /*MSP430TimerC.MSP430TimerB4*/MSP430TimerCapComM$7$Control$disableEvents(void );
#line 32
static void /*MSP430TimerC.MSP430TimerB4*/MSP430TimerCapComM$7$Control$clearPendingInterrupt(void );
# 4 "/Users/jingyuancheng/tinyos/moteiv/tos/platform/msp430/timer/MSP430TimerEvent.nc"
static void /*MSP430TimerC.MSP430TimerB4*/MSP430TimerCapComM$7$Event$fired(void );
# 30 "/Users/jingyuancheng/tinyos/moteiv/tos/platform/msp430/timer/MSP430Compare.nc"
static void /*MSP430TimerC.MSP430TimerB4*/MSP430TimerCapComM$7$Compare$setEvent(uint16_t time);

static void /*MSP430TimerC.MSP430TimerB4*/MSP430TimerCapComM$7$Compare$setEventFromNow(uint16_t delta);
# 33 "/Users/jingyuancheng/tinyos/moteiv/tos/platform/msp430/timer/MSP430Timer.nc"
static void /*MSP430TimerC.MSP430TimerB4*/MSP430TimerCapComM$7$Timer$overflow(void );
# 32 "/Users/jingyuancheng/tinyos/moteiv/tos/platform/msp430/timer/MSP430Capture.nc"
static uint16_t /*MSP430TimerC.MSP430TimerB5*/MSP430TimerCapComM$8$Capture$getEvent(void );
#line 74
static void /*MSP430TimerC.MSP430TimerB5*/MSP430TimerCapComM$8$Capture$default$captured(uint16_t time);
# 30 "/Users/jingyuancheng/tinyos/moteiv/tos/platform/msp430/timer/MSP430TimerControl.nc"
static MSP430CompareControl_t /*MSP430TimerC.MSP430TimerB5*/MSP430TimerCapComM$8$Control$getControl(void );
# 4 "/Users/jingyuancheng/tinyos/moteiv/tos/platform/msp430/timer/MSP430TimerEvent.nc"
static void /*MSP430TimerC.MSP430TimerB5*/MSP430TimerCapComM$8$Event$fired(void );
# 34 "/Users/jingyuancheng/tinyos/moteiv/tos/platform/msp430/timer/MSP430Compare.nc"
static void /*MSP430TimerC.MSP430TimerB5*/MSP430TimerCapComM$8$Compare$default$fired(void );
# 33 "/Users/jingyuancheng/tinyos/moteiv/tos/platform/msp430/timer/MSP430Timer.nc"
static void /*MSP430TimerC.MSP430TimerB5*/MSP430TimerCapComM$8$Timer$overflow(void );
# 32 "/Users/jingyuancheng/tinyos/moteiv/tos/platform/msp430/timer/MSP430Capture.nc"
static uint16_t /*MSP430TimerC.MSP430TimerB6*/MSP430TimerCapComM$9$Capture$getEvent(void );
#line 74
static void /*MSP430TimerC.MSP430TimerB6*/MSP430TimerCapComM$9$Capture$default$captured(uint16_t time);
# 30 "/Users/jingyuancheng/tinyos/moteiv/tos/platform/msp430/timer/MSP430TimerControl.nc"
static MSP430CompareControl_t /*MSP430TimerC.MSP430TimerB6*/MSP430TimerCapComM$9$Control$getControl(void );
# 4 "/Users/jingyuancheng/tinyos/moteiv/tos/platform/msp430/timer/MSP430TimerEvent.nc"
static void /*MSP430TimerC.MSP430TimerB6*/MSP430TimerCapComM$9$Event$fired(void );
# 34 "/Users/jingyuancheng/tinyos/moteiv/tos/platform/msp430/timer/MSP430Compare.nc"
static void /*MSP430TimerC.MSP430TimerB6*/MSP430TimerCapComM$9$Compare$default$fired(void );
# 33 "/Users/jingyuancheng/tinyos/moteiv/tos/platform/msp430/timer/MSP430Timer.nc"
static void /*MSP430TimerC.MSP430TimerB6*/MSP430TimerCapComM$9$Timer$overflow(void );
# 40 "/Users/jingyuancheng/tinyos/moteiv/tos/lib/sched/Boot.nc"
static void MainP$Boot$default$booted(void );
# 63 "/Users/jingyuancheng/tinyos/moteiv/tinyos-1.x/tos/interfaces/StdControl.nc"
static result_t MainP$MainStdControl$default$init(
# 12 "/Users/jingyuancheng/tinyos/moteiv/tos/lib/sched/MainP.nc"
uint8_t arg_0x101d29020);
# 70 "/Users/jingyuancheng/tinyos/moteiv/tinyos-1.x/tos/interfaces/StdControl.nc"
static result_t MainP$MainStdControl$default$start(
# 12 "/Users/jingyuancheng/tinyos/moteiv/tos/lib/sched/MainP.nc"
uint8_t arg_0x101d29020);
# 64 "/Users/jingyuancheng/tinyos/moteiv/tinyos-1.x/tos/interfaces/SplitControl.nc"
static result_t MainP$MainSplitControl$default$init(
# 11 "/Users/jingyuancheng/tinyos/moteiv/tos/lib/sched/MainP.nc"
uint8_t arg_0x101d2caa8);
# 77 "/Users/jingyuancheng/tinyos/moteiv/tinyos-1.x/tos/interfaces/SplitControl.nc"
static result_t MainP$MainSplitControl$default$start(
# 11 "/Users/jingyuancheng/tinyos/moteiv/tos/lib/sched/MainP.nc"
uint8_t arg_0x101d2caa8);
# 46 "/Users/jingyuancheng/tinyos/moteiv/tos/lib/sched/Init.nc"
static result_t MainP$MainInit$default$init(
# 13 "/Users/jingyuancheng/tinyos/moteiv/tos/lib/sched/MainP.nc"
uint8_t arg_0x101d28020);
# 133 "/Users/jingyuancheng/tinyos/moteiv/tos/platform/msp430/adc/MSP430ADC12Single.nc"
static result_t ADC8IO14P$ADC6$dataReady(uint16_t data);
#line 133
static result_t ADC8IO14P$ADC0$dataReady(uint16_t data);
#line 133
static result_t ADC8IO14P$ADC3$dataReady(uint16_t data);
#line 133
static result_t ADC8IO14P$ADC4$dataReady(uint16_t data);
#line 133
static result_t ADC8IO14P$ADC7$dataReady(uint16_t data);
#line 133
static result_t ADC8IO14P$ADC1$dataReady(uint16_t data);
#line 133
static result_t ADC8IO14P$ADC5$dataReady(uint16_t data);
# 49 "/Users/jingyuancheng/tinyos/moteiv/tinyos-1.x/tos/interfaces/SendMsg.nc"
static result_t ADC8IO14P$SendMsg$sendDone(TOS_MsgPtr msg, result_t success);
# 133 "/Users/jingyuancheng/tinyos/moteiv/tos/platform/msp430/adc/MSP430ADC12Single.nc"
static result_t ADC8IO14P$ADC2$dataReady(uint16_t data);
# 63 "/Users/jingyuancheng/tinyos/moteiv/tinyos-1.x/tos/interfaces/StdControl.nc"
static result_t ADC8IO14P$StdControl$init(void );






static result_t ADC8IO14P$StdControl$start(void );
# 58 "/Users/jingyuancheng/tinyos/moteiv/tos/lib/sched/TaskBasic.nc"
static void ADC8IO14P$taskSendData$runTask(void );
# 68 "/Users/jingyuancheng/tinyos/moteiv/tos/lib/timer/Timer2.nc"
static void ADC8IO14P$Timer$fired(void );
# 33 "/Users/jingyuancheng/tinyos/moteiv/tos/platform/msp430/timer/MSP430Timer.nc"
static void /*HalTimerMilliC.AlarmMilliC.MSP430Alarm*/MSP430AlarmC$0$MSP430Timer$overflow(void );
# 34 "/Users/jingyuancheng/tinyos/moteiv/tos/platform/msp430/timer/MSP430Compare.nc"
static void /*HalTimerMilliC.AlarmMilliC.MSP430Alarm*/MSP430AlarmC$0$MSP430Compare$fired(void );
# 88 "/Users/jingyuancheng/tinyos/moteiv/tos/lib/timer/Alarm.nc"
static void /*HalTimerMilliC.AlarmMilliC.MSP430Alarm*/MSP430AlarmC$0$Alarm$startAt(/*HalTimerMilliC.AlarmMilliC.MSP430Alarm*/MSP430AlarmC$0$Alarm$size_type t0, /*HalTimerMilliC.AlarmMilliC.MSP430Alarm*/MSP430AlarmC$0$Alarm$size_type dt);
#line 60
static void /*HalTimerMilliC.AlarmMilliC.MSP430Alarm*/MSP430AlarmC$0$Alarm$stop(void );
# 63 "/Users/jingyuancheng/tinyos/moteiv/tinyos-1.x/tos/interfaces/StdControl.nc"
static result_t /*HalTimerMilliC.AlarmMilliC.MSP430Alarm*/MSP430AlarmC$0$Init$init(void );






static result_t /*HalTimerMilliC.AlarmMilliC.MSP430Alarm*/MSP430AlarmC$0$Init$start(void );
# 93 "/Users/jingyuancheng/tinyos/moteiv/tos/lib/timer/Alarm.nc"
static /*HalTimerMilliC.AlarmMilliC.Transform*/TransformAlarmC$0$Alarm$size_type /*HalTimerMilliC.AlarmMilliC.Transform*/TransformAlarmC$0$Alarm$getNow(void );
#line 88
static void /*HalTimerMilliC.AlarmMilliC.Transform*/TransformAlarmC$0$Alarm$startAt(/*HalTimerMilliC.AlarmMilliC.Transform*/TransformAlarmC$0$Alarm$size_type t0, /*HalTimerMilliC.AlarmMilliC.Transform*/TransformAlarmC$0$Alarm$size_type dt);
#line 60
static void /*HalTimerMilliC.AlarmMilliC.Transform*/TransformAlarmC$0$Alarm$stop(void );



static void /*HalTimerMilliC.AlarmMilliC.Transform*/TransformAlarmC$0$AlarmFrom$fired(void );
# 70 "/Users/jingyuancheng/tinyos/moteiv/tos/lib/timer/Counter.nc"
static void /*HalTimerMilliC.AlarmMilliC.Transform*/TransformAlarmC$0$Counter$overflow(void );
# 33 "/Users/jingyuancheng/tinyos/moteiv/tos/platform/msp430/timer/MSP430Timer.nc"
static void /*MSP430Counter32khzC.Counter*/MSP430CounterC$0$MSP430Timer$overflow(void );
# 52 "/Users/jingyuancheng/tinyos/moteiv/tos/lib/timer/Counter.nc"
static /*MSP430Counter32khzC.Counter*/MSP430CounterC$0$Counter$size_type /*MSP430Counter32khzC.Counter*/MSP430CounterC$0$Counter$get(void );






static bool /*MSP430Counter32khzC.Counter*/MSP430CounterC$0$Counter$isOverflowPending(void );










static void /*CounterMilliC.Transform*/TransformCounterC$0$CounterFrom$overflow(void );
#line 52
static /*CounterMilliC.Transform*/TransformCounterC$0$Counter$size_type /*CounterMilliC.Transform*/TransformCounterC$0$Counter$get(void );
#line 70
static void /*CounterMilliC.CounterToLocalTimeC*/CounterToLocalTimeC$0$Counter$overflow(void );
# 58 "/Users/jingyuancheng/tinyos/moteiv/tos/lib/sched/TaskBasic.nc"
static void /*HalTimerMilliC.AlarmToTimerC*/AlarmToTimerC$0$fired$runTask(void );
# 64 "/Users/jingyuancheng/tinyos/moteiv/tos/lib/timer/Alarm.nc"
static void /*HalTimerMilliC.AlarmToTimerC*/AlarmToTimerC$0$Alarm$fired(void );
# 116 "/Users/jingyuancheng/tinyos/moteiv/tos/lib/timer/Timer2.nc"
static uint32_t /*HalTimerMilliC.AlarmToTimerC*/AlarmToTimerC$0$Timer$getNow(void );
#line 129
static uint32_t /*HalTimerMilliC.AlarmToTimerC*/AlarmToTimerC$0$Timer$getdt(void );
#line 123
static uint32_t /*HalTimerMilliC.AlarmToTimerC*/AlarmToTimerC$0$Timer$gett0(void );
#line 111
static void /*HalTimerMilliC.AlarmToTimerC*/AlarmToTimerC$0$Timer$startOneShotAt(uint32_t t0, uint32_t dt);
#line 64
static void /*HalTimerMilliC.AlarmToTimerC*/AlarmToTimerC$0$Timer$stop(void );



static void /*HalTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC$0$TimerFrom$fired(void );
# 63 "/Users/jingyuancheng/tinyos/moteiv/tinyos-1.x/tos/interfaces/StdControl.nc"
static result_t /*HalTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC$0$Init$init(void );






static result_t /*HalTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC$0$Init$start(void );
# 58 "/Users/jingyuancheng/tinyos/moteiv/tos/lib/sched/TaskBasic.nc"
static void /*HalTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC$0$executeTimersNow$runTask(void );
# 52 "/Users/jingyuancheng/tinyos/moteiv/tos/lib/timer/Timer2.nc"
static void /*HalTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC$0$Timer$startPeriodic(
# 30 "/Users/jingyuancheng/tinyos/moteiv/tos/lib/timer/VirtualizeTimerC.nc"
uint8_t arg_0x101f29108, 
# 52 "/Users/jingyuancheng/tinyos/moteiv/tos/lib/timer/Timer2.nc"
uint32_t dt);







static void /*HalTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC$0$Timer$startOneShot(
# 30 "/Users/jingyuancheng/tinyos/moteiv/tos/lib/timer/VirtualizeTimerC.nc"
uint8_t arg_0x101f29108, 
# 60 "/Users/jingyuancheng/tinyos/moteiv/tos/lib/timer/Timer2.nc"
uint32_t dt);



static void /*HalTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC$0$Timer$stop(
# 30 "/Users/jingyuancheng/tinyos/moteiv/tos/lib/timer/VirtualizeTimerC.nc"
uint8_t arg_0x101f29108);
# 73 "/Users/jingyuancheng/tinyos/moteiv/tinyos-1.x/tos/interfaces/Timer.nc"
static result_t FramerP$Timer$fired(void );
# 33 "/Users/jingyuancheng/tinyos/moteiv/tos/interfaces/Detect.nc"
static void FramerP$Detect$disconnected(void );
#line 28
static void FramerP$Detect$connected(void );
# 58 "/Users/jingyuancheng/tinyos/moteiv/tos/lib/sched/TaskBasic.nc"
static void FramerP$PacketUnknown$runTask(void );
#line 58
static void FramerP$PacketSent$runTask(void );
# 83 "/Users/jingyuancheng/tinyos/moteiv/tinyos-1.x/tos/interfaces/ByteComm.nc"
static result_t FramerP$ByteComm$txDone(void );
#line 75
static result_t FramerP$ByteComm$txByteReady(bool success);
#line 66
static result_t FramerP$ByteComm$rxByteReady(uint8_t data, bool error, uint16_t strength);
# 58 "/Users/jingyuancheng/tinyos/moteiv/tos/lib/sched/TaskBasic.nc"
static void FramerP$PacketRcvd$runTask(void );
# 58 "/Users/jingyuancheng/tinyos/moteiv/tinyos-1.x/tos/interfaces/BareSendMsg.nc"
static result_t FramerP$BareSendMsg$send(TOS_MsgPtr msg);
# 63 "/Users/jingyuancheng/tinyos/moteiv/tinyos-1.x/tos/interfaces/StdControl.nc"
static result_t FramerP$StdControl$init(void );






static result_t FramerP$StdControl$start(void );
# 88 "/Users/jingyuancheng/tinyos/moteiv/tinyos-1.x/tos/interfaces/TokenReceiveMsg.nc"
static result_t FramerP$TokenReceiveMsg$ReflectToken(uint8_t Token);
# 115 "/Users/jingyuancheng/tinyos/moteiv/tinyos-1.x/tos/platform/msp430/HPLUSARTControl.nc"
static void HPLUSART1M$USARTControl$disableSPI(void );
#line 169
static void HPLUSART1M$USARTControl$setClockRate(uint16_t baudrate, uint8_t mctl);
#line 100
static void HPLUSART1M$USARTControl$enableUARTRx(void );
#line 85
static void HPLUSART1M$USARTControl$disableUART(void );
#line 167
static void HPLUSART1M$USARTControl$setClockSource(uint8_t source);






static result_t HPLUSART1M$USARTControl$enableRxIntr(void );
static result_t HPLUSART1M$USARTControl$enableTxIntr(void );
#line 90
static void HPLUSART1M$USARTControl$enableUARTTx(void );
#line 105
static void HPLUSART1M$USARTControl$disableUARTRx(void );
#line 125
static void HPLUSART1M$USARTControl$disableI2C(void );
#line 202
static result_t HPLUSART1M$USARTControl$tx(uint8_t data);
#line 153
static void HPLUSART1M$USARTControl$setModeUART(void );
#line 95
static void HPLUSART1M$USARTControl$disableUARTTx(void );
# 58 "/Users/jingyuancheng/tinyos/moteiv/tos/lib/sched/TaskBasic.nc"
static void UartPresenceM$taskConnected$runTask(void );
# 23 "/Users/jingyuancheng/tinyos/moteiv/tos/interfaces/Detect.nc"
static bool UartPresenceM$Presence$isConnected(void );
# 58 "/Users/jingyuancheng/tinyos/moteiv/tos/lib/sched/TaskBasic.nc"
static void UartPresenceM$taskDisconnected$runTask(void );
# 63 "/Users/jingyuancheng/tinyos/moteiv/tinyos-1.x/tos/interfaces/StdControl.nc"
static result_t UartPresenceM$StdControl$init(void );






static result_t UartPresenceM$StdControl$start(void );
# 63 "/Users/jingyuancheng/tinyos/moteiv/tos/platform/msp430/MSP430Interrupt.nc"
static void UartPresenceM$Interrupt$fired(void );
#line 44
static void MSP430InterruptM$Port14$clear(void );
#line 69
static void MSP430InterruptM$Port14$makeInput(void );
#line 39
static void MSP430InterruptM$Port14$disable(void );
#line 58
static void MSP430InterruptM$Port14$edge(bool low_to_high);
#line 34
static void MSP430InterruptM$Port14$enable(void );









static void MSP430InterruptM$Port26$clear(void );
#line 63
static void MSP430InterruptM$Port26$default$fired(void );
#line 44
static void MSP430InterruptM$Port17$clear(void );
#line 63
static void MSP430InterruptM$Port17$default$fired(void );
#line 44
static void MSP430InterruptM$Port21$clear(void );
#line 63
static void MSP430InterruptM$Port21$default$fired(void );
#line 44
static void MSP430InterruptM$Port12$clear(void );
#line 39
static void MSP430InterruptM$Port12$disable(void );
#line 58
static void MSP430InterruptM$Port12$edge(bool low_to_high);
#line 34
static void MSP430InterruptM$Port12$enable(void );









static void MSP430InterruptM$Port24$clear(void );
#line 63
static void MSP430InterruptM$Port24$default$fired(void );
#line 44
static void MSP430InterruptM$ACCV$clear(void );
#line 63
static void MSP430InterruptM$ACCV$default$fired(void );
#line 44
static void MSP430InterruptM$Port15$clear(void );
#line 63
static void MSP430InterruptM$Port15$default$fired(void );
#line 44
static void MSP430InterruptM$Port27$clear(void );
#line 63
static void MSP430InterruptM$Port27$default$fired(void );
#line 44
static void MSP430InterruptM$Port10$clear(void );
#line 69
static void MSP430InterruptM$Port10$makeInput(void );
#line 39
static void MSP430InterruptM$Port10$disable(void );
#line 58
static void MSP430InterruptM$Port10$edge(bool low_to_high);
#line 34
static void MSP430InterruptM$Port10$enable(void );









static void MSP430InterruptM$Port22$clear(void );
#line 63
static void MSP430InterruptM$Port22$default$fired(void );
#line 44
static void MSP430InterruptM$OF$clear(void );
#line 63
static void MSP430InterruptM$OF$default$fired(void );
#line 44
static void MSP430InterruptM$Port13$clear(void );
#line 69
static void MSP430InterruptM$Port13$makeInput(void );
#line 39
static void MSP430InterruptM$Port13$disable(void );




static void MSP430InterruptM$Port25$clear(void );
#line 63
static void MSP430InterruptM$Port25$default$fired(void );
#line 44
static void MSP430InterruptM$Port16$clear(void );
#line 63
static void MSP430InterruptM$Port16$default$fired(void );
#line 44
static void MSP430InterruptM$NMI$clear(void );
#line 63
static void MSP430InterruptM$NMI$default$fired(void );
#line 44
static void MSP430InterruptM$Port20$clear(void );
#line 63
static void MSP430InterruptM$Port20$default$fired(void );
#line 44
static void MSP430InterruptM$Port11$clear(void );
#line 63
static void MSP430InterruptM$Port11$default$fired(void );
#line 44
static void MSP430InterruptM$Port23$clear(void );
#line 63
static void MSP430InterruptM$Port23$default$fired(void );
# 33 "/Users/jingyuancheng/tinyos/moteiv/tinyos-1.x/tos/platform/msp430/MSP430GeneralIO.nc"
static void MSP430GeneralIOM$Port26$makeOutput(void );
#line 27
static void MSP430GeneralIOM$Port26$setHigh(void );
static void MSP430GeneralIOM$Port26$setLow(void );




static void MSP430GeneralIOM$Port54$makeOutput(void );
#line 27
static void MSP430GeneralIOM$Port54$setHigh(void );
static void MSP430GeneralIOM$Port54$setLow(void );




static void MSP430GeneralIOM$Port21$makeOutput(void );
#line 27
static void MSP430GeneralIOM$Port21$setHigh(void );
static void MSP430GeneralIOM$Port21$setLow(void );


static bool MSP430GeneralIOM$Port12$get(void );

static void MSP430GeneralIOM$Port15$makeOutput(void );
#line 27
static void MSP430GeneralIOM$Port15$setHigh(void );
static void MSP430GeneralIOM$Port15$setLow(void );




static void MSP430GeneralIOM$Port55$makeOutput(void );
#line 27
static void MSP430GeneralIOM$Port55$setHigh(void );
static void MSP430GeneralIOM$Port55$setLow(void );




static void MSP430GeneralIOM$Port16$makeOutput(void );
#line 27
static void MSP430GeneralIOM$Port16$setHigh(void );
static void MSP430GeneralIOM$Port16$setLow(void );




static void MSP430GeneralIOM$Port20$makeOutput(void );
#line 27
static void MSP430GeneralIOM$Port20$setHigh(void );
static void MSP430GeneralIOM$Port20$setLow(void );




static void MSP430GeneralIOM$Port23$makeOutput(void );
#line 27
static void MSP430GeneralIOM$Port23$setHigh(void );
static void MSP430GeneralIOM$Port23$setLow(void );




static void MSP430GeneralIOM$Port56$makeOutput(void );
#line 27
static void MSP430GeneralIOM$Port56$setHigh(void );
static void MSP430GeneralIOM$Port56$setLow(void );
# 43 "/Users/jingyuancheng/tinyos/moteiv/tos/lib/timer/TimerMilli.nc"
static result_t TimerWrapC$TimerMilli$default$fired(
# 9 "/Users/jingyuancheng/tinyos/moteiv/tos/lib/timer/TimerWrapC.nc"
uint8_t arg_0x1025b6ab8);
# 68 "/Users/jingyuancheng/tinyos/moteiv/tos/lib/timer/Timer2.nc"
static void TimerWrapC$Timer2$fired(
# 10 "/Users/jingyuancheng/tinyos/moteiv/tos/lib/timer/TimerWrapC.nc"
uint8_t arg_0x1025b4258);
# 73 "/Users/jingyuancheng/tinyos/moteiv/tinyos-1.x/tos/interfaces/Timer.nc"
static result_t TimerWrapC$Timer$default$fired(
# 8 "/Users/jingyuancheng/tinyos/moteiv/tos/lib/timer/TimerWrapC.nc"
uint8_t arg_0x1025bac10);
# 59 "/Users/jingyuancheng/tinyos/moteiv/tinyos-1.x/tos/interfaces/Timer.nc"
static result_t TimerWrapC$Timer$start(
# 8 "/Users/jingyuancheng/tinyos/moteiv/tos/lib/timer/TimerWrapC.nc"
uint8_t arg_0x1025bac10, 
# 59 "/Users/jingyuancheng/tinyos/moteiv/tinyos-1.x/tos/interfaces/Timer.nc"
char type, uint32_t interval);








static result_t TimerWrapC$Timer$stop(
# 8 "/Users/jingyuancheng/tinyos/moteiv/tos/lib/timer/TimerWrapC.nc"
uint8_t arg_0x1025bac10);
# 75 "/Users/jingyuancheng/tinyos/moteiv/tinyos-1.x/tos/interfaces/ReceiveMsg.nc"
static TOS_MsgPtr FramerAckM$ReceiveMsg$receive(TOS_MsgPtr m);
# 58 "/Users/jingyuancheng/tinyos/moteiv/tos/lib/sched/TaskBasic.nc"
static void FramerAckM$SendAckTask$runTask(void );
# 75 "/Users/jingyuancheng/tinyos/moteiv/tinyos-1.x/tos/interfaces/TokenReceiveMsg.nc"
static TOS_MsgPtr FramerAckM$TokenReceiveMsg$receive(TOS_MsgPtr Msg, uint8_t Token);
# 88 "/Users/jingyuancheng/tinyos/moteiv/tinyos-1.x/tos/interfaces/HPLUART.nc"
static result_t UARTM$HPLUART$get(uint8_t data);







static result_t UARTM$HPLUART$putDone(void );
# 55 "/Users/jingyuancheng/tinyos/moteiv/tinyos-1.x/tos/interfaces/ByteComm.nc"
static result_t UARTM$ByteComm$txByte(uint8_t data);
# 63 "/Users/jingyuancheng/tinyos/moteiv/tinyos-1.x/tos/interfaces/StdControl.nc"
static result_t UARTM$Control$init(void );






static result_t UARTM$Control$start(void );
# 53 "/Users/jingyuancheng/tinyos/moteiv/tinyos-1.x/tos/platform/msp430/HPLUSARTFeedback.nc"
static result_t HPLUARTM$USARTData$rxDone(uint8_t data);
#line 46
static result_t HPLUARTM$USARTData$txDone(void );
# 62 "/Users/jingyuancheng/tinyos/moteiv/tinyos-1.x/tos/interfaces/HPLUART.nc"
static result_t HPLUARTM$UART$init(void );
#line 80
static result_t HPLUARTM$UART$put(uint8_t data);
# 19 "/Users/jingyuancheng/tinyos/moteiv/tos/lib/CC2420Radio/MacBackoff.nc"
static int16_t CC2420AlwaysOnM$MacBackoff$initialBackoff(TOS_MsgPtr m);
static int16_t CC2420AlwaysOnM$MacBackoff$congestionBackoff(TOS_MsgPtr m);
# 70 "/Users/jingyuancheng/tinyos/moteiv/tinyos-1.x/tos/interfaces/SplitControl.nc"
static result_t CC2420AlwaysOnM$RadioControl$initDone(void );
#line 85
static result_t CC2420AlwaysOnM$RadioControl$startDone(void );
#line 99
static result_t CC2420AlwaysOnM$RadioControl$stopDone(void );
# 64 "/Users/jingyuancheng/tinyos/moteiv/tos/lib/timer/Alarm.nc"
static void CC2420AlwaysOnM$AlarmStart$fired(void );
# 68 "/Users/jingyuancheng/tinyos/moteiv/tos/lib/timer/Timer2.nc"
static void CC2420AlwaysOnM$SanityTimer$fired(void );
# 68 "/Users/jingyuancheng/tinyos/moteiv/tos/lib/CC2420Radio/CC2420BareSendMsg.nc"
static result_t CC2420AlwaysOnM$LowerSend$sendDone(TOS_MsgPtr msg, cc2420_error_t success);
# 64 "/Users/jingyuancheng/tinyos/moteiv/tos/lib/timer/Alarm.nc"
static void CC2420AlwaysOnM$AlarmStop$fired(void );
# 90 "/Users/jingyuancheng/tinyos/moteiv/tos/lib/sp/SPSend.nc"
static result_t CC2420AlwaysOnM$SPSend$sendAdv(sp_message_t *msg, TOS_Msg *tosmsg, sp_device_t dev, sp_address_t addr, uint8_t length, sp_message_flags_t flags, uint8_t quantity);
# 63 "/Users/jingyuancheng/tinyos/moteiv/tinyos-1.x/tos/interfaces/StdControl.nc"
static result_t CC2420AlwaysOnM$StdControl$init(void );






static result_t CC2420AlwaysOnM$StdControl$start(void );
# 23 "/Users/jingyuancheng/tinyos/moteiv/tos/lib/sp/SPLinkStats.nc"
static sp_linkstate_t CC2420AlwaysOnM$SPLinkStats$getState(void );
# 64 "/Users/jingyuancheng/tinyos/moteiv/tinyos-1.x/tos/interfaces/SplitControl.nc"
static result_t CC2420RadioM$SplitControl$init(void );
#line 77
static result_t CC2420RadioM$SplitControl$start(void );
# 58 "/Users/jingyuancheng/tinyos/moteiv/tos/lib/sched/TaskBasic.nc"
static void CC2420RadioM$sendFailedTask$runTask(void );
# 51 "/Users/jingyuancheng/tinyos/moteiv/tinyos-1.x/tos/lib/CC2420Radio/HPLCC2420Interrupt.nc"
static result_t CC2420RadioM$FIFOP$fired(void );
# 80 "/Users/jingyuancheng/tinyos/moteiv/tos/lib/resource/ResourceCmdAsync.nc"
static void CC2420RadioM$CmdFlushRXFIFO$granted(uint8_t rh);
# 59 "/Users/jingyuancheng/tinyos/moteiv/tos/lib/CC2420Radio/CC2420BareSendMsg.nc"
static result_t CC2420RadioM$Send$send(TOS_MsgPtr msg);
# 58 "/Users/jingyuancheng/tinyos/moteiv/tos/lib/sched/TaskBasic.nc"
static void CC2420RadioM$startRadio$runTask(void );
# 31 "/Users/jingyuancheng/tinyos/moteiv/tos/lib/CC2420Radio/MacControl.nc"
static void CC2420RadioM$MacControl$requestAck(TOS_MsgPtr msg);
#line 22
static void CC2420RadioM$MacControl$enableAck(void );
# 64 "/Users/jingyuancheng/tinyos/moteiv/tos/lib/timer/Alarm.nc"
static void CC2420RadioM$BackoffAlarm32khz$fired(void );
# 70 "/Users/jingyuancheng/tinyos/moteiv/tos/lib/resource/ResourceCmd.nc"
static void CC2420RadioM$CmdTransmit$granted(uint8_t rh);
# 58 "/Users/jingyuancheng/tinyos/moteiv/tos/lib/sched/TaskBasic.nc"
static void CC2420RadioM$PacketSent$runTask(void );
#line 58
static void CC2420RadioM$PacketRcvd$runTask(void );
# 53 "/Users/jingyuancheng/tinyos/moteiv/tinyos-1.x/tos/lib/CC2420Radio/HPLCC2420Capture.nc"
static result_t CC2420RadioM$SFD$captured(uint16_t val);
# 70 "/Users/jingyuancheng/tinyos/moteiv/tos/lib/timer/Counter.nc"
static void CC2420RadioM$RadioActiveTime$default$overflow(void );
# 70 "/Users/jingyuancheng/tinyos/moteiv/tos/lib/resource/ResourceCmd.nc"
static void CC2420RadioM$CmdTryToSend$granted(uint8_t rh);
# 70 "/Users/jingyuancheng/tinyos/moteiv/tos/lib/timer/Counter.nc"
static void CC2420RadioM$Counter32khz$overflow(void );
# 63 "/Users/jingyuancheng/tinyos/moteiv/tos/lib/CC2420Radio/HPLCC2420FIFO.nc"
static result_t CC2420RadioM$HPLChipconFIFO$TXFIFODone(uint8_t length, uint8_t *data);
#line 52
static result_t CC2420RadioM$HPLChipconFIFO$RXFIFODone(uint8_t length, uint8_t *data);
# 70 "/Users/jingyuancheng/tinyos/moteiv/tos/lib/resource/ResourceCmd.nc"
static void CC2420RadioM$CmdReceive$granted(uint8_t rh);
# 70 "/Users/jingyuancheng/tinyos/moteiv/tos/lib/timer/Counter.nc"
static void CC2420RadioM$Counter32khz16$overflow(void );
# 58 "/Users/jingyuancheng/tinyos/moteiv/tos/lib/sched/TaskBasic.nc"
static void CC2420RadioM$taskShutdownRequest$runTask(void );
# 70 "/Users/jingyuancheng/tinyos/moteiv/tinyos-1.x/tos/interfaces/SplitControl.nc"
static result_t CC2420RadioM$CC2420SplitControl$initDone(void );
#line 85
static result_t CC2420RadioM$CC2420SplitControl$startDone(void );
#line 99
static result_t CC2420RadioM$CC2420SplitControl$stopDone(void );
#line 64
static result_t CC2420ControlM$SplitControl$init(void );
#line 77
static result_t CC2420ControlM$SplitControl$start(void );
#line 93
static result_t CC2420ControlM$SplitControl$stop(void );
# 70 "/Users/jingyuancheng/tinyos/moteiv/tos/lib/resource/ResourceCmd.nc"
static void CC2420ControlM$CmdCCAFired$granted(uint8_t rh);
#line 70
static void CC2420ControlM$CmdSplitControlStop$granted(uint8_t rh);
# 51 "/Users/jingyuancheng/tinyos/moteiv/tinyos-1.x/tos/lib/CC2420Radio/HPLCC2420Interrupt.nc"
static result_t CC2420ControlM$CCA$fired(void );
# 70 "/Users/jingyuancheng/tinyos/moteiv/tos/lib/resource/ResourceCmd.nc"
static void CC2420ControlM$CmdSplitControlInit$granted(uint8_t rh);
#line 70
static void CC2420ControlM$CmdSplitControlStart$granted(uint8_t rh);
# 51 "/Users/jingyuancheng/tinyos/moteiv/tos/lib/CC2420Radio/HPLCC2420RAM.nc"
static result_t CC2420ControlM$HPLChipconRAM$writeDone(uint16_t addr, uint8_t length, uint8_t *buffer);
#line 66
static result_t CC2420ControlM$HPLChipconRAM$readDone(uint16_t addr, uint8_t length, uint8_t *buffer);
# 114 "/Users/jingyuancheng/tinyos/moteiv/tos/lib/CC2420Radio/CC2420Control.nc"
static result_t CC2420ControlM$CC2420Control$VREFOn(void );
#line 172
static result_t CC2420ControlM$CC2420Control$RxMode(uint8_t rh);
#line 88
static result_t CC2420ControlM$CC2420Control$TuneManual(uint8_t rh, uint16_t freq);
#line 247
static result_t CC2420ControlM$CC2420Control$setShortAddress(uint8_t rh, uint16_t addr);
#line 121
static result_t CC2420ControlM$CC2420Control$VREFOff(void );









static result_t CC2420ControlM$CC2420Control$OscillatorOn(uint8_t rh);
# 80 "/Users/jingyuancheng/tinyos/moteiv/tos/lib/resource/ResourceCmdAsync.nc"
static void CC2420ControlM$CmdCmds$granted(uint8_t rh);
# 58 "/Users/jingyuancheng/tinyos/moteiv/tos/lib/sched/TaskBasic.nc"
static void HPLCC2420M$signalRXFIFO$runTask(void );
#line 58
static void HPLCC2420M$signalRAMWr$runTask(void );
#line 58
static void HPLCC2420M$signalTXFIFO$runTask(void );
# 71 "/Users/jingyuancheng/tinyos/moteiv/tos/lib/CC2420Radio/HPLCC2420.nc"
static uint16_t HPLCC2420M$HPLCC2420$read(uint8_t rh, uint8_t addr);
#line 60
static uint8_t HPLCC2420M$HPLCC2420$write(uint8_t rh, uint8_t addr, uint16_t data);
#line 48
static uint8_t HPLCC2420M$HPLCC2420$cmd(uint8_t rh, uint8_t addr);
# 42 "/Users/jingyuancheng/tinyos/moteiv/tos/lib/CC2420Radio/HPLCC2420FIFO.nc"
static result_t HPLCC2420M$HPLCC2420FIFO$writeTXFIFO(uint8_t rh, uint8_t length, uint8_t *data);
#line 30
static result_t HPLCC2420M$HPLCC2420FIFO$readRXFIFO(uint8_t rh, uint8_t length, uint8_t *data);
# 49 "/Users/jingyuancheng/tinyos/moteiv/tos/lib/CC2420Radio/HPLCC2420RAM.nc"
static result_t HPLCC2420M$HPLCC2420RAM$write(uint8_t rh, uint16_t addr, uint8_t length, uint8_t *buffer);
# 63 "/Users/jingyuancheng/tinyos/moteiv/tinyos-1.x/tos/interfaces/StdControl.nc"
static result_t HPLCC2420M$StdControl$init(void );






static result_t HPLCC2420M$StdControl$start(void );







static result_t HPLCC2420M$StdControl$stop(void );
# 58 "/Users/jingyuancheng/tinyos/moteiv/tos/lib/sched/TaskBasic.nc"
static void HPLCC2420M$signalRAMRd$runTask(void );
# 43 "/Users/jingyuancheng/tinyos/moteiv/tinyos-1.x/tos/platform/msp430/HPLI2CInterrupt.nc"
static void HPLUSART0M$HPLI2CInterrupt$default$fired(void );
# 53 "/Users/jingyuancheng/tinyos/moteiv/tinyos-1.x/tos/platform/msp430/HPLUSARTFeedback.nc"
static result_t HPLUSART0M$USARTData$default$rxDone(uint8_t data);
#line 46
static result_t HPLUSART0M$USARTData$default$txDone(void );
# 191 "/Users/jingyuancheng/tinyos/moteiv/tinyos-1.x/tos/platform/msp430/HPLUSARTControl.nc"
static result_t HPLUSART0M$USARTControl$isTxEmpty(void );
#line 115
static void HPLUSART0M$USARTControl$disableSPI(void );
#line 85
static void HPLUSART0M$USARTControl$disableUART(void );
#line 159
static bool HPLUSART0M$USARTControl$isI2C(void );
#line 172
static result_t HPLUSART0M$USARTControl$disableRxIntr(void );
static result_t HPLUSART0M$USARTControl$disableTxIntr(void );
#line 125
static void HPLUSART0M$USARTControl$disableI2C(void );









static void HPLUSART0M$USARTControl$setModeSPI(void );
#line 180
static result_t HPLUSART0M$USARTControl$isTxIntrPending(void );
#line 202
static result_t HPLUSART0M$USARTControl$tx(uint8_t data);






static uint8_t HPLUSART0M$USARTControl$rx(void );
#line 185
static result_t HPLUSART0M$USARTControl$isRxIntrPending(void );
# 51 "/Users/jingyuancheng/tinyos/moteiv/tinyos-1.x/tos/lib/CC2420Radio/HPLCC2420Interrupt.nc"
static result_t HPLCC2420InterruptM$FIFO$default$fired(void );







static result_t HPLCC2420InterruptM$FIFOP$disable(void );
#line 43
static result_t HPLCC2420InterruptM$FIFOP$startWait(bool low_to_high);
# 63 "/Users/jingyuancheng/tinyos/moteiv/tos/platform/msp430/MSP430Interrupt.nc"
static void HPLCC2420InterruptM$CCAInterrupt$fired(void );
# 46 "/Users/jingyuancheng/tinyos/moteiv/tos/lib/sched/Init.nc"
static result_t HPLCC2420InterruptM$Init$init(void );
# 59 "/Users/jingyuancheng/tinyos/moteiv/tinyos-1.x/tos/lib/CC2420Radio/HPLCC2420Interrupt.nc"
static result_t HPLCC2420InterruptM$CCA$disable(void );
#line 43
static result_t HPLCC2420InterruptM$CCA$startWait(bool low_to_high);
# 63 "/Users/jingyuancheng/tinyos/moteiv/tos/platform/msp430/MSP430Interrupt.nc"
static void HPLCC2420InterruptM$FIFOInterrupt$fired(void );
# 60 "/Users/jingyuancheng/tinyos/moteiv/tinyos-1.x/tos/lib/CC2420Radio/HPLCC2420Capture.nc"
static result_t HPLCC2420InterruptM$SFD$disable(void );
#line 43
static result_t HPLCC2420InterruptM$SFD$enableCapture(bool low_to_high);
# 74 "/Users/jingyuancheng/tinyos/moteiv/tos/platform/msp430/timer/MSP430Capture.nc"
static void HPLCC2420InterruptM$SFDCapture$captured(uint16_t time);
# 63 "/Users/jingyuancheng/tinyos/moteiv/tos/platform/msp430/MSP430Interrupt.nc"
static void HPLCC2420InterruptM$FIFOPInterrupt$fired(void );
# 75 "/Users/jingyuancheng/tinyos/moteiv/tos/lib/resource/ResourceCmd.nc"
static void /*MSP430ArbiterUSART0C.ArbiterC.ArbiterP*/FcfsArbiterP$1$ResourceCmd$release(
# 22 "/Users/jingyuancheng/tinyos/moteiv/tos/lib/resource/FcfsArbiterP.nc"
uint8_t arg_0x101b3b660);
# 63 "/Users/jingyuancheng/tinyos/moteiv/tos/lib/resource/ResourceCmd.nc"
static void /*MSP430ArbiterUSART0C.ArbiterC.ArbiterP*/FcfsArbiterP$1$ResourceCmd$deferRequest(
# 22 "/Users/jingyuancheng/tinyos/moteiv/tos/lib/resource/FcfsArbiterP.nc"
uint8_t arg_0x101b3b660);
# 70 "/Users/jingyuancheng/tinyos/moteiv/tos/lib/resource/ResourceCmd.nc"
static void /*MSP430ArbiterUSART0C.ArbiterC.ArbiterP*/FcfsArbiterP$1$ResourceCmd$default$granted(
# 22 "/Users/jingyuancheng/tinyos/moteiv/tos/lib/resource/FcfsArbiterP.nc"
uint8_t arg_0x101b3b660, 
# 70 "/Users/jingyuancheng/tinyos/moteiv/tos/lib/resource/ResourceCmd.nc"
uint8_t rh);
# 29 "/Users/jingyuancheng/tinyos/moteiv/tos/lib/resource/ResourceConfigure.nc"
static void /*MSP430ArbiterUSART0C.ArbiterC.ArbiterP*/FcfsArbiterP$1$ResourceConfigure$default$configure(
# 24 "/Users/jingyuancheng/tinyos/moteiv/tos/lib/resource/FcfsArbiterP.nc"
uint8_t arg_0x101b719b0);
# 28 "/Users/jingyuancheng/tinyos/moteiv/tos/lib/resource/ResourceValidate.nc"
static bool /*MSP430ArbiterUSART0C.ArbiterC.ArbiterP*/FcfsArbiterP$1$ResourceValidate$validateUser(uint8_t rh);
# 65 "/Users/jingyuancheng/tinyos/moteiv/tos/lib/resource/ResourceCmdAsync.nc"
static void /*MSP430ArbiterUSART0C.ArbiterC.ArbiterP*/FcfsArbiterP$1$ResourceCmdAsync$urgentRequest(
# 23 "/Users/jingyuancheng/tinyos/moteiv/tos/lib/resource/FcfsArbiterP.nc"
uint8_t arg_0x101b3a698, 
# 65 "/Users/jingyuancheng/tinyos/moteiv/tos/lib/resource/ResourceCmdAsync.nc"
uint8_t rh);
#line 85
static void /*MSP430ArbiterUSART0C.ArbiterC.ArbiterP*/FcfsArbiterP$1$ResourceCmdAsync$release(
# 23 "/Users/jingyuancheng/tinyos/moteiv/tos/lib/resource/FcfsArbiterP.nc"
uint8_t arg_0x101b3a698);
# 58 "/Users/jingyuancheng/tinyos/moteiv/tos/lib/resource/ResourceCmdAsync.nc"
static void /*MSP430ArbiterUSART0C.ArbiterC.ArbiterP*/FcfsArbiterP$1$ResourceCmdAsync$request(
# 23 "/Users/jingyuancheng/tinyos/moteiv/tos/lib/resource/FcfsArbiterP.nc"
uint8_t arg_0x101b3a698, 
# 58 "/Users/jingyuancheng/tinyos/moteiv/tos/lib/resource/ResourceCmdAsync.nc"
uint8_t rh);
#line 80
static void /*MSP430ArbiterUSART0C.ArbiterC.ArbiterP*/FcfsArbiterP$1$ResourceCmdAsync$default$granted(
# 23 "/Users/jingyuancheng/tinyos/moteiv/tos/lib/resource/FcfsArbiterP.nc"
uint8_t arg_0x101b3a698, 
# 80 "/Users/jingyuancheng/tinyos/moteiv/tos/lib/resource/ResourceCmdAsync.nc"
uint8_t rh);
# 46 "/Users/jingyuancheng/tinyos/moteiv/tos/lib/sched/Init.nc"
static result_t /*MSP430ArbiterUSART0C.ArbiterC.ArbiterP*/FcfsArbiterP$1$Init$init(void );
# 58 "/Users/jingyuancheng/tinyos/moteiv/tos/lib/sched/TaskBasic.nc"
static void /*MSP430ArbiterUSART0C.ArbiterC.ArbiterP*/FcfsArbiterP$1$GrantTask$runTask(void );
# 78 "/Users/jingyuancheng/tinyos/moteiv/tos/lib/resource/Resource.nc"
static void /*MSP430ArbiterUSART0C.ArbiterC.ArbiterP*/FcfsArbiterP$1$Resource$release(
# 21 "/Users/jingyuancheng/tinyos/moteiv/tos/lib/resource/FcfsArbiterP.nc"
uint8_t arg_0x101b3c648);
# 58 "/Users/jingyuancheng/tinyos/moteiv/tos/lib/resource/Resource.nc"
static void /*MSP430ArbiterUSART0C.ArbiterC.ArbiterP*/FcfsArbiterP$1$Resource$request(
# 21 "/Users/jingyuancheng/tinyos/moteiv/tos/lib/resource/FcfsArbiterP.nc"
uint8_t arg_0x101b3c648);
# 73 "/Users/jingyuancheng/tinyos/moteiv/tos/lib/resource/Resource.nc"
static void /*MSP430ArbiterUSART0C.ArbiterC.ArbiterP*/FcfsArbiterP$1$Resource$default$granted(
# 21 "/Users/jingyuancheng/tinyos/moteiv/tos/lib/resource/FcfsArbiterP.nc"
uint8_t arg_0x101b3c648);
# 54 "/Users/jingyuancheng/tinyos/moteiv/tos/lib/resource/Arbiter.nc"
static void /*MSP430ResourceConfigUSART0C.ConfigUSARTP*/MSP430ResourceConfigUSARTP$0$Arbiter$requested(void );





static void /*MSP430ResourceConfigUSART0C.ConfigUSARTP*/MSP430ResourceConfigUSARTP$0$Arbiter$idle(void );
# 29 "/Users/jingyuancheng/tinyos/moteiv/tos/lib/resource/ResourceConfigure.nc"
static void /*MSP430ResourceConfigUSART0C.ConfigUSARTP*/MSP430ResourceConfigUSARTP$0$ConfigSPI$configure(void );
# 63 "/Users/jingyuancheng/tinyos/moteiv/tinyos-1.x/tos/interfaces/Random.nc"
static uint16_t RandomMLCG$Random$rand(void );
#line 57
static result_t RandomMLCG$Random$init(void );
# 33 "/Users/jingyuancheng/tinyos/moteiv/tos/platform/msp430/timer/MSP430Timer.nc"
static void /*CC2420RadioC.BackoffAlarm32khzC.MSP430Alarm*/MSP430AlarmC$1$MSP430Timer$overflow(void );
# 34 "/Users/jingyuancheng/tinyos/moteiv/tos/platform/msp430/timer/MSP430Compare.nc"
static void /*CC2420RadioC.BackoffAlarm32khzC.MSP430Alarm*/MSP430AlarmC$1$MSP430Compare$fired(void );
# 93 "/Users/jingyuancheng/tinyos/moteiv/tos/lib/timer/Alarm.nc"
static /*CC2420RadioC.BackoffAlarm32khzC.MSP430Alarm*/MSP430AlarmC$1$Alarm$size_type /*CC2420RadioC.BackoffAlarm32khzC.MSP430Alarm*/MSP430AlarmC$1$Alarm$getNow(void );
#line 88
static void /*CC2420RadioC.BackoffAlarm32khzC.MSP430Alarm*/MSP430AlarmC$1$Alarm$startAt(/*CC2420RadioC.BackoffAlarm32khzC.MSP430Alarm*/MSP430AlarmC$1$Alarm$size_type t0, /*CC2420RadioC.BackoffAlarm32khzC.MSP430Alarm*/MSP430AlarmC$1$Alarm$size_type dt);
#line 74
static bool /*CC2420RadioC.BackoffAlarm32khzC.MSP430Alarm*/MSP430AlarmC$1$Alarm$isRunning(void );
#line 54
static void /*CC2420RadioC.BackoffAlarm32khzC.MSP430Alarm*/MSP430AlarmC$1$Alarm$start(/*CC2420RadioC.BackoffAlarm32khzC.MSP430Alarm*/MSP430AlarmC$1$Alarm$size_type dt);





static void /*CC2420RadioC.BackoffAlarm32khzC.MSP430Alarm*/MSP430AlarmC$1$Alarm$stop(void );
# 63 "/Users/jingyuancheng/tinyos/moteiv/tinyos-1.x/tos/interfaces/StdControl.nc"
static result_t /*CC2420RadioC.BackoffAlarm32khzC.MSP430Alarm*/MSP430AlarmC$1$Init$init(void );






static result_t /*CC2420RadioC.BackoffAlarm32khzC.MSP430Alarm*/MSP430AlarmC$1$Init$start(void );







static result_t /*CC2420RadioC.BackoffAlarm32khzC.MSP430Alarm*/MSP430AlarmC$1$Init$stop(void );
# 70 "/Users/jingyuancheng/tinyos/moteiv/tos/lib/timer/Counter.nc"
static void /*Counter32khzC.Transform*/TransformCounterC$1$CounterFrom$overflow(void );
#line 52
static /*Counter32khzC.Transform*/TransformCounterC$1$Counter$size_type /*Counter32khzC.Transform*/TransformCounterC$1$Counter$get(void );
# 49 "/Users/jingyuancheng/tinyos/moteiv/tos/lib/timer/LocalTime.nc"
static uint32_t /*Counter32khzC.CounterToLocalTimeC*/CounterToLocalTimeC$1$LocalTime$get(void );
# 70 "/Users/jingyuancheng/tinyos/moteiv/tos/lib/timer/Counter.nc"
static void /*Counter32khzC.CounterToLocalTimeC*/CounterToLocalTimeC$1$Counter$overflow(void );
# 80 "/Users/jingyuancheng/tinyos/moteiv/tos/lib/resource/ResourceCmdAsync.nc"
static void CC2420TimeStampingM$CmdWriteTimeStamp$granted(uint8_t rh);
# 33 "/Users/jingyuancheng/tinyos/moteiv/tinyos-1.x/tos/interfaces/RadioCoordinator.nc"
static void CC2420TimeStampingM$RadioReceiveCoordinator$startSymbol(uint8_t bitsPerBlock, uint8_t offset, TOS_MsgPtr msgBuff);
#line 33
static void CC2420TimeStampingM$RadioSendCoordinator$startSymbol(uint8_t bitsPerBlock, uint8_t offset, TOS_MsgPtr msgBuff);
# 51 "/Users/jingyuancheng/tinyos/moteiv/tos/lib/CC2420Radio/HPLCC2420RAM.nc"
static result_t CC2420TimeStampingM$HPLCC2420RAM$writeDone(uint16_t addr, uint8_t length, uint8_t *buffer);
#line 66
static result_t CC2420TimeStampingM$HPLCC2420RAM$readDone(uint16_t addr, uint8_t length, uint8_t *buffer);
# 79 "/Users/jingyuancheng/tinyos/moteiv/tos/lib/sp/cc2420/TimeStamping.nc"
static result_t CC2420TimeStampingM$TimeStamping$addStamp(TOS_MsgPtr msg, int8_t offset);
# 64 "/Users/jingyuancheng/tinyos/moteiv/tos/lib/timer/Alarm.nc"
static void /*CC2420SyncAlwaysOnC.DualAlarmC*/VirtualizeAlarmC$0$Alarm$default$fired(
# 38 "/Users/jingyuancheng/tinyos/moteiv/tos/lib/timer/VirtualizeAlarmC.nc"
uint8_t arg_0x102b44b20);
# 63 "/Users/jingyuancheng/tinyos/moteiv/tinyos-1.x/tos/interfaces/StdControl.nc"
static result_t /*CC2420SyncAlwaysOnC.DualAlarmC*/VirtualizeAlarmC$0$Init$init(void );






static result_t /*CC2420SyncAlwaysOnC.DualAlarmC*/VirtualizeAlarmC$0$Init$start(void );
# 64 "/Users/jingyuancheng/tinyos/moteiv/tos/lib/timer/Alarm.nc"
static void /*CC2420SyncAlwaysOnC.DualAlarmC*/VirtualizeAlarmC$0$AlarmFrom$fired(void );
# 33 "/Users/jingyuancheng/tinyos/moteiv/tos/platform/msp430/timer/MSP430Timer.nc"
static void /*CC2420SyncAlwaysOnC.AlarmC.MSP430Alarm*/MSP430AlarmC$2$MSP430Timer$overflow(void );
# 34 "/Users/jingyuancheng/tinyos/moteiv/tos/platform/msp430/timer/MSP430Compare.nc"
static void /*CC2420SyncAlwaysOnC.AlarmC.MSP430Alarm*/MSP430AlarmC$2$MSP430Compare$fired(void );
# 88 "/Users/jingyuancheng/tinyos/moteiv/tos/lib/timer/Alarm.nc"
static void /*CC2420SyncAlwaysOnC.AlarmC.MSP430Alarm*/MSP430AlarmC$2$Alarm$startAt(/*CC2420SyncAlwaysOnC.AlarmC.MSP430Alarm*/MSP430AlarmC$2$Alarm$size_type t0, /*CC2420SyncAlwaysOnC.AlarmC.MSP430Alarm*/MSP430AlarmC$2$Alarm$size_type dt);
#line 60
static void /*CC2420SyncAlwaysOnC.AlarmC.MSP430Alarm*/MSP430AlarmC$2$Alarm$stop(void );
# 63 "/Users/jingyuancheng/tinyos/moteiv/tinyos-1.x/tos/interfaces/StdControl.nc"
static result_t /*CC2420SyncAlwaysOnC.AlarmC.MSP430Alarm*/MSP430AlarmC$2$Init$init(void );






static result_t /*CC2420SyncAlwaysOnC.AlarmC.MSP430Alarm*/MSP430AlarmC$2$Init$start(void );
# 93 "/Users/jingyuancheng/tinyos/moteiv/tos/lib/timer/Alarm.nc"
static /*CC2420SyncAlwaysOnC.AlarmC.Transform*/TransformAlarmC$1$Alarm$size_type /*CC2420SyncAlwaysOnC.AlarmC.Transform*/TransformAlarmC$1$Alarm$getNow(void );
#line 88
static void /*CC2420SyncAlwaysOnC.AlarmC.Transform*/TransformAlarmC$1$Alarm$startAt(/*CC2420SyncAlwaysOnC.AlarmC.Transform*/TransformAlarmC$1$Alarm$size_type t0, /*CC2420SyncAlwaysOnC.AlarmC.Transform*/TransformAlarmC$1$Alarm$size_type dt);
#line 60
static void /*CC2420SyncAlwaysOnC.AlarmC.Transform*/TransformAlarmC$1$Alarm$stop(void );



static void /*CC2420SyncAlwaysOnC.AlarmC.Transform*/TransformAlarmC$1$AlarmFrom$fired(void );
# 70 "/Users/jingyuancheng/tinyos/moteiv/tos/lib/timer/Counter.nc"
static void /*CC2420SyncAlwaysOnC.AlarmC.Transform*/TransformAlarmC$1$Counter$overflow(void );
# 62 "/Users/jingyuancheng/tinyos/moteiv/tos/lib/util/pool/ObjectPool.nc"
static /*SPC.MessagePool*/ObjectPoolC$0$Pool$object_type */*SPC.MessagePool*/ObjectPoolC$0$Pool$get(uint8_t position);
#line 33
static result_t /*SPC.MessagePool*/ObjectPoolC$0$Pool$remove(/*SPC.MessagePool*/ObjectPoolC$0$Pool$object_type *obj);
#line 24
static result_t /*SPC.MessagePool*/ObjectPoolC$0$Pool$insert(/*SPC.MessagePool*/ObjectPoolC$0$Pool$object_type *obj);
#line 71
static bool /*SPC.MessagePool*/ObjectPoolC$0$Pool$valid(uint8_t n);
static uint8_t /*SPC.MessagePool*/ObjectPoolC$0$Pool$next(uint8_t n);
#line 69
static uint8_t /*SPC.MessagePool*/ObjectPoolC$0$Pool$first(void );
#line 62
static /*SPC.NeighborTable*/ObjectPoolC$1$Pool$object_type */*SPC.NeighborTable*/ObjectPoolC$1$Pool$get(uint8_t position);
#line 47
static uint8_t /*SPC.NeighborTable*/ObjectPoolC$1$Pool$populated(void );
#line 71
static bool /*SPC.NeighborTable*/ObjectPoolC$1$Pool$valid(uint8_t n);
static uint8_t /*SPC.NeighborTable*/ObjectPoolC$1$Pool$next(uint8_t n);
#line 69
static uint8_t /*SPC.NeighborTable*/ObjectPoolC$1$Pool$first(void );
# 75 "/Users/jingyuancheng/tinyos/moteiv/tinyos-1.x/tos/interfaces/ReceiveMsg.nc"
static TOS_MsgPtr SPM$UARTReceive$receive(TOS_MsgPtr m);
#line 75
static TOS_MsgPtr SPM$ReceiveMsg$default$receive(
# 25 "/Users/jingyuancheng/tinyos/moteiv/tos/lib/sp/SPM.nc"
uint8_t arg_0x102c5c618, 
# 75 "/Users/jingyuancheng/tinyos/moteiv/tinyos-1.x/tos/interfaces/ReceiveMsg.nc"
TOS_MsgPtr m);
#line 75
static TOS_MsgPtr SPM$LowerReceive$receive(TOS_MsgPtr m);
# 26 "/Users/jingyuancheng/tinyos/moteiv/tos/lib/util/pool/ObjectPoolEvents.nc"
static void SPM$PoolEvents$removed(SPM$PoolEvents$object_type *object);
#line 20
static void SPM$PoolEvents$inserted(SPM$PoolEvents$object_type *object);
# 40 "/Users/jingyuancheng/tinyos/moteiv/tos/lib/sp/SPReceive.nc"
static void SPM$SPReceive$default$receive(
# 24 "/Users/jingyuancheng/tinyos/moteiv/tos/lib/sp/SPM.nc"
uint8_t arg_0x102c5dae8, 
# 40 "/Users/jingyuancheng/tinyos/moteiv/tos/lib/sp/SPReceive.nc"
sp_message_t *spmsg, TOS_MsgPtr tosmsg, sp_error_t result);
# 126 "/Users/jingyuancheng/tinyos/moteiv/tos/lib/sp/SPSend.nc"
static void SPM$SPDataMgr$sendDone(
# 28 "/Users/jingyuancheng/tinyos/moteiv/tos/lib/sp/SPM.nc"
uint8_t arg_0x102c5a258, 
# 126 "/Users/jingyuancheng/tinyos/moteiv/tos/lib/sp/SPSend.nc"
sp_message_t *msg, sp_message_flags_t flags, sp_error_t error);
# 41 "/Users/jingyuancheng/tinyos/moteiv/tos/lib/sp/SPSendNext.nc"
static void SPM$SPDataMgrNext$request(
# 29 "/Users/jingyuancheng/tinyos/moteiv/tos/lib/sp/SPM.nc"
uint8_t arg_0x102c59490, 
# 41 "/Users/jingyuancheng/tinyos/moteiv/tos/lib/sp/SPSendNext.nc"
sp_message_t *msg, TOS_Msg *tosmsg, uint8_t remaining);
#line 41
static void SPM$SPSendNext$default$request(
# 23 "/Users/jingyuancheng/tinyos/moteiv/tos/lib/sp/SPM.nc"
uint8_t arg_0x102c5ee00, 
# 41 "/Users/jingyuancheng/tinyos/moteiv/tos/lib/sp/SPSendNext.nc"
sp_message_t *msg, TOS_Msg *tosmsg, uint8_t remaining);
# 126 "/Users/jingyuancheng/tinyos/moteiv/tos/lib/sp/SPSend.nc"
static void SPM$LowerSend$sendDone(sp_message_t *msg, sp_message_flags_t flags, sp_error_t error);
#line 46
static result_t SPM$SPSend$send(
# 22 "/Users/jingyuancheng/tinyos/moteiv/tos/lib/sp/SPM.nc"
uint8_t arg_0x102c62ca8, 
# 46 "/Users/jingyuancheng/tinyos/moteiv/tos/lib/sp/SPSend.nc"
sp_message_t *msg, TOS_Msg *tosmsg, sp_address_t addr, uint8_t length);
#line 90
static result_t SPM$SPSend$sendAdv(
# 22 "/Users/jingyuancheng/tinyos/moteiv/tos/lib/sp/SPM.nc"
uint8_t arg_0x102c62ca8, 
# 90 "/Users/jingyuancheng/tinyos/moteiv/tos/lib/sp/SPSend.nc"
sp_message_t *msg, TOS_Msg *tosmsg, sp_device_t dev, sp_address_t addr, uint8_t length, sp_message_flags_t flags, uint8_t quantity);
# 67 "/Users/jingyuancheng/tinyos/moteiv/tinyos-1.x/tos/interfaces/BareSendMsg.nc"
static result_t SPDataM$UARTSend$sendDone(TOS_MsgPtr msg, result_t success);
# 90 "/Users/jingyuancheng/tinyos/moteiv/tos/lib/sp/SPSend.nc"
static result_t SPDataM$SPSend$sendAdv(
# 18 "/Users/jingyuancheng/tinyos/moteiv/tos/lib/sp/SPDataM.nc"
uint8_t arg_0x102cbc0c8, 
# 90 "/Users/jingyuancheng/tinyos/moteiv/tos/lib/sp/SPSend.nc"
sp_message_t *msg, TOS_Msg *tosmsg, sp_device_t dev, sp_address_t addr, uint8_t length, sp_message_flags_t flags, uint8_t quantity);
# 22 "/Users/jingyuancheng/tinyos/moteiv/tos/lib/sp/SPNeighbor.nc"
static sp_neighbor_t *SPNeighborTableM$SPNeighbor$get(
# 17 "/Users/jingyuancheng/tinyos/moteiv/tos/lib/sp/SPNeighborTableM.nc"
uint8_t arg_0x102cc8db0, 
# 22 "/Users/jingyuancheng/tinyos/moteiv/tos/lib/sp/SPNeighbor.nc"
uint8_t n);









static uint8_t SPNeighborTableM$SPNeighbor$populated(
# 17 "/Users/jingyuancheng/tinyos/moteiv/tos/lib/sp/SPNeighborTableM.nc"
uint8_t arg_0x102cc8db0);
# 30 "/Users/jingyuancheng/tinyos/moteiv/tos/lib/sp/SPNeighbor.nc"
static bool SPNeighborTableM$SPNeighbor$valid(
# 17 "/Users/jingyuancheng/tinyos/moteiv/tos/lib/sp/SPNeighborTableM.nc"
uint8_t arg_0x102cc8db0, 
# 30 "/Users/jingyuancheng/tinyos/moteiv/tos/lib/sp/SPNeighbor.nc"
uint8_t n);
static uint8_t SPNeighborTableM$SPNeighbor$next(
# 17 "/Users/jingyuancheng/tinyos/moteiv/tos/lib/sp/SPNeighborTableM.nc"
uint8_t arg_0x102cc8db0, 
# 31 "/Users/jingyuancheng/tinyos/moteiv/tos/lib/sp/SPNeighbor.nc"
uint8_t n);
#line 29
static uint8_t SPNeighborTableM$SPNeighbor$first(
# 17 "/Users/jingyuancheng/tinyos/moteiv/tos/lib/sp/SPNeighborTableM.nc"
uint8_t arg_0x102cc8db0);
# 48 "/Users/jingyuancheng/tinyos/moteiv/tinyos-1.x/tos/interfaces/SendMsg.nc"
static result_t SPAdaptorGenericCommM$SendMsg$send(
# 19 "/Users/jingyuancheng/tinyos/moteiv/tos/lib/sp/SPAdaptorGenericCommM.nc"
uint8_t arg_0x102da0790, 
# 48 "/Users/jingyuancheng/tinyos/moteiv/tinyos-1.x/tos/interfaces/SendMsg.nc"
uint16_t address, uint8_t length, TOS_MsgPtr msg);
static result_t SPAdaptorGenericCommM$SendMsg$default$sendDone(
# 19 "/Users/jingyuancheng/tinyos/moteiv/tos/lib/sp/SPAdaptorGenericCommM.nc"
uint8_t arg_0x102da0790, 
# 49 "/Users/jingyuancheng/tinyos/moteiv/tinyos-1.x/tos/interfaces/SendMsg.nc"
TOS_MsgPtr msg, result_t success);
# 126 "/Users/jingyuancheng/tinyos/moteiv/tos/lib/sp/SPSend.nc"
static void SPAdaptorGenericCommM$SPSend$sendDone(
# 22 "/Users/jingyuancheng/tinyos/moteiv/tos/lib/sp/SPAdaptorGenericCommM.nc"
uint8_t arg_0x102d9f488, 
# 126 "/Users/jingyuancheng/tinyos/moteiv/tos/lib/sp/SPSend.nc"
sp_message_t *msg, sp_message_flags_t flags, sp_error_t error);
# 127 "/Users/jingyuancheng/tinyos/moteiv/tos/platform/msp430/adc/RefVolt.nc"
static void MSP430ADC12M$RefVolt$isStable(RefVolt_t vref);
# 67 "/Users/jingyuancheng/tinyos/moteiv/tos/platform/msp430/adc/MSP430ADC12Single.nc"
static msp430ADCresult_t MSP430ADC12M$ADCSingle$getData(
# 41 "/Users/jingyuancheng/tinyos/moteiv/tos/platform/msp430/adc/MSP430ADC12M.nc"
uint8_t arg_0x102dc9578);
# 52 "/Users/jingyuancheng/tinyos/moteiv/tos/platform/msp430/adc/MSP430ADC12Single.nc"
static result_t MSP430ADC12M$ADCSingle$bind(
# 41 "/Users/jingyuancheng/tinyos/moteiv/tos/platform/msp430/adc/MSP430ADC12M.nc"
uint8_t arg_0x102dc9578, 
# 52 "/Users/jingyuancheng/tinyos/moteiv/tos/platform/msp430/adc/MSP430ADC12Single.nc"
MSP430ADC12Settings_t settings);
#line 133
static result_t MSP430ADC12M$ADCSingle$default$dataReady(
# 41 "/Users/jingyuancheng/tinyos/moteiv/tos/platform/msp430/adc/MSP430ADC12M.nc"
uint8_t arg_0x102dc9578, 
# 133 "/Users/jingyuancheng/tinyos/moteiv/tos/platform/msp430/adc/MSP430ADC12Single.nc"
uint16_t data);
# 168 "/Users/jingyuancheng/tinyos/moteiv/tos/platform/msp430/adc/MSP430ADC12Multiple.nc"
static uint16_t *MSP430ADC12M$ADCMultiple$default$dataReady(
# 42 "/Users/jingyuancheng/tinyos/moteiv/tos/platform/msp430/adc/MSP430ADC12M.nc"
uint8_t arg_0x102dc6220, 
# 168 "/Users/jingyuancheng/tinyos/moteiv/tos/platform/msp430/adc/MSP430ADC12Multiple.nc"
uint16_t *buf, uint16_t length);
# 61 "/Users/jingyuancheng/tinyos/moteiv/tos/platform/msp430/adc/HPLADC12.nc"
static void MSP430ADC12M$HPLADC12$memOverflow(void );

static void MSP430ADC12M$HPLADC12$converted(uint8_t number);
#line 62
static void MSP430ADC12M$HPLADC12$timeOverflow(void );
# 80 "/Users/jingyuancheng/tinyos/moteiv/tos/lib/resource/ResourceCmdAsync.nc"
static void MSP430ADC12M$Resource$granted(uint8_t rh);
# 63 "/Users/jingyuancheng/tinyos/moteiv/tinyos-1.x/tos/interfaces/StdControl.nc"
static result_t MSP430ADC12M$StdControl$init(void );






static result_t MSP430ADC12M$StdControl$start(void );
# 73 "/Users/jingyuancheng/tinyos/moteiv/tos/platform/msp430/adc/HPLADC12.nc"
static void HPLADC12M$HPLADC12$setRefOff(void );
#line 58
static void HPLADC12M$HPLADC12$resetIFGs(void );






static bool HPLADC12M$HPLADC12$isBusy(void );
#line 43
static void HPLADC12M$HPLADC12$setControl1(adc12ctl1_t control1);
#line 76
static void HPLADC12M$HPLADC12$setRef2_5V(void );



static void HPLADC12M$HPLADC12$disableConversion(void );
#line 48
static void HPLADC12M$HPLADC12$setControl0_IgnoreRef(adc12ctl0_t control0);
#line 72
static void HPLADC12M$HPLADC12$setRefOn(void );
#line 51
static adc12memctl_t HPLADC12M$HPLADC12$getMemControl(uint8_t i);
#line 75
static void HPLADC12M$HPLADC12$setRef1_5V(void );





static void HPLADC12M$HPLADC12$startConversion(void );
#line 52
static uint16_t HPLADC12M$HPLADC12$getMem(uint8_t i);


static void HPLADC12M$HPLADC12$setIEFlags(uint16_t mask);
#line 69
static void HPLADC12M$HPLADC12$setSHT(uint8_t sht);
#line 50
static void HPLADC12M$HPLADC12$setMemControl(uint8_t index, adc12memctl_t memControl);
#line 82
static void HPLADC12M$HPLADC12$stopConversion(void );
# 18 "/Users/jingyuancheng/tinyos/moteiv/tos/platform/msp430/timer/TimerExclusive.nc"
static result_t MSP430TimerAExclusiveM$TimerExclusive$startTimer(uint8_t rh);



static result_t MSP430TimerAExclusiveM$TimerExclusive$stopTimer(uint8_t rh);
#line 14
static result_t MSP430TimerAExclusiveM$TimerExclusive$prepareTimer(uint8_t rh, uint16_t interval, uint16_t csSAMPCON, uint16_t cdSAMPCON);
# 34 "/Users/jingyuancheng/tinyos/moteiv/tos/platform/msp430/timer/MSP430Compare.nc"
static void MSP430TimerAExclusiveM$CompareA1$fired(void );
# 33 "/Users/jingyuancheng/tinyos/moteiv/tos/platform/msp430/timer/MSP430Timer.nc"
static void MSP430TimerAExclusiveM$TimerA$overflow(void );
# 34 "/Users/jingyuancheng/tinyos/moteiv/tos/platform/msp430/timer/MSP430Compare.nc"
static void MSP430TimerAExclusiveM$CompareA0$fired(void );
# 58 "/Users/jingyuancheng/tinyos/moteiv/tos/lib/sched/TaskBasic.nc"
static void RefVoltM$switchOnDelay$runTask(void );
# 68 "/Users/jingyuancheng/tinyos/moteiv/tos/lib/timer/Timer2.nc"
static void RefVoltM$SwitchOffTimer$fired(void );
# 118 "/Users/jingyuancheng/tinyos/moteiv/tos/platform/msp430/adc/RefVolt.nc"
static RefVolt_t RefVoltM$RefVolt$getState(void );
#line 109
static result_t RefVoltM$RefVolt$release(void );
#line 92
static result_t RefVoltM$RefVolt$get(RefVolt_t vref);
# 58 "/Users/jingyuancheng/tinyos/moteiv/tos/lib/sched/TaskBasic.nc"
static void RefVoltM$switchOffDelay$runTask(void );
# 61 "/Users/jingyuancheng/tinyos/moteiv/tos/platform/msp430/adc/HPLADC12.nc"
static void RefVoltM$HPLADC12$memOverflow(void );

static void RefVoltM$HPLADC12$converted(uint8_t number);
#line 62
static void RefVoltM$HPLADC12$timeOverflow(void );
# 58 "/Users/jingyuancheng/tinyos/moteiv/tos/lib/sched/TaskBasic.nc"
static void RefVoltM$switchOffRetry$runTask(void );
# 68 "/Users/jingyuancheng/tinyos/moteiv/tos/lib/timer/Timer2.nc"
static void RefVoltM$SwitchOnTimer$fired(void );
# 46 "/Users/jingyuancheng/tinyos/moteiv/tos/lib/sched/Init.nc"
static result_t PlatformP$ArbiterInits$init(void );
# 8 "/Users/jingyuancheng/tinyos/moteiv/tos/platform/msp430/PlatformP.nc"
static result_t PlatformP$hplInit(void );



static inline result_t PlatformP$HPLInit$init(void );
# 63 "/Users/jingyuancheng/tinyos/moteiv/tinyos-1.x/tos/interfaces/StdControl.nc"
static result_t HPLInitM$MSP430ClockControl$init(void );






static result_t HPLInitM$MSP430ClockControl$start(void );
# 35 "/Users/jingyuancheng/tinyos/moteiv/tinyos-1.x/tos/platform/msp430/HPLInitM.nc"
static inline result_t HPLInitM$init(void );
# 29 "/Users/jingyuancheng/tinyos/moteiv/tos/platform/msp430/timer/MSP430ClockInit.nc"
static void MSP430ClockM$MSP430ClockInit$initTimerB(void );
#line 28
static void MSP430ClockM$MSP430ClockInit$initTimerA(void );
#line 27
static void MSP430ClockM$MSP430ClockInit$initClocks(void );
# 34 "/Users/jingyuancheng/tinyos/moteiv/tos/platform/msp430/timer/MSP430ClockM.nc"
typedef result_t MSP430ClockM$error_t;

static volatile uint8_t MSP430ClockM$IE1 __asm ("0x0000");
static volatile uint16_t MSP430ClockM$TA0CTL __asm ("0x0160");
static volatile uint16_t MSP430ClockM$TA0IV __asm ("0x012E");
static volatile uint16_t MSP430ClockM$TBCTL __asm ("0x0180");
static volatile uint16_t MSP430ClockM$TBIV __asm ("0x011E");

enum MSP430ClockM$__nesc_unnamed4332 {

  MSP430ClockM$ACLK_CALIB_PERIOD = 8, 
  MSP430ClockM$ACLK_KHZ = 32, 



  MSP430ClockM$TARGET_DCO_KHZ = 4096, 

  MSP430ClockM$TARGET_DCO_DELTA = MSP430ClockM$TARGET_DCO_KHZ / MSP430ClockM$ACLK_KHZ * MSP430ClockM$ACLK_CALIB_PERIOD
};

static inline void MSP430ClockM$MSP430ClockInit$defaultInitClocks(void );
#line 81
static inline void MSP430ClockM$MSP430ClockInit$defaultInitTimerA(void );
#line 96
static inline void MSP430ClockM$MSP430ClockInit$defaultInitTimerB(void );
#line 111
static inline void MSP430ClockM$MSP430ClockInit$default$initClocks(void );




static inline void MSP430ClockM$MSP430ClockInit$default$initTimerA(void );




static inline void MSP430ClockM$MSP430ClockInit$default$initTimerB(void );





static inline void MSP430ClockM$startTimerA(void );
#line 139
static inline void MSP430ClockM$startTimerB(void );
#line 151
static void MSP430ClockM$set_dco_calib(int calib);





static inline uint16_t MSP430ClockM$test_calib_busywait_delta(int calib);
#line 180
static inline void MSP430ClockM$busyCalibrateDCO(void );
#line 217
static inline MSP430ClockM$error_t MSP430ClockM$Init$init(void );
#line 238
static inline result_t MSP430ClockM$Init$start(void );
# 39 "/Users/jingyuancheng/tinyos/moteiv/tos/platform/msp430/timer/MSP430TimerControl.nc"
static void MSP430DCOCalibP$TimerControlB$disableEvents(void );
#line 31
static bool MSP430DCOCalibP$TimerControlB$isInterruptPending(void );
static void MSP430DCOCalibP$TimerControlB$clearPendingInterrupt(void );
# 40 "/Users/jingyuancheng/tinyos/moteiv/tos/platform/msp430/timer/MSP430Timer.nc"
static void MSP430DCOCalibP$TimerA$setClockSource(uint16_t clockSource);
#line 30
static uint16_t MSP430DCOCalibP$TimerA$get(void );








static void MSP430DCOCalibP$TimerA$disableEvents(void );
#line 35
static void MSP430DCOCalibP$TimerA$setMode(int mode);





static void MSP430DCOCalibP$TimerA$setInputDivider(uint16_t inputDivider);
# 31 "/Users/jingyuancheng/tinyos/moteiv/tos/platform/msp430/timer/MSP430Compare.nc"
static void MSP430DCOCalibP$TimerCompareB$setEventFromPrev(uint16_t delta);
static void MSP430DCOCalibP$TimerCompareB$setEventFromNow(uint16_t delta);
# 75 "/Users/jingyuancheng/tinyos/moteiv/tos/lib/resource/ResourceCmd.nc"
static void MSP430DCOCalibP$ResourceTimerA$release(void );
#line 63
static void MSP430DCOCalibP$ResourceTimerA$deferRequest(void );
# 25 "/Users/jingyuancheng/tinyos/moteiv/tos/platform/msp430/timer/MSP430DCOCalibP.nc"
enum MSP430DCOCalibP$__nesc_unnamed4333 {
  MSP430DCOCalibP$MEASURE_DELTA_RTC = 12, 
  MSP430DCOCalibP$TARGET_DELTA_SMCLK = 384, 
  MSP430DCOCalibP$MAX_SMCLK_DEVIATION = 2
};


static inline void MSP430DCOCalibP$TimerA$overflow(void );



static inline void MSP430DCOCalibP$TimerB$overflow(void );



static inline void MSP430DCOCalibP$TimerCompareB$fired(void );


static inline uint16_t MSP430DCOCalibP$get_delta_dco(void );
#line 63
static inline void MSP430DCOCalibP$step_dco(uint16_t td_dco);
#line 89
static inline void MSP430DCOCalibP$ResourceTimerA$granted(uint8_t rh);
# 70 "/Users/jingyuancheng/tinyos/moteiv/tos/lib/resource/ResourceCmd.nc"
static void /*MSP430ArbiterTimerAC.ArbiterC.ArbiterP*/FcfsArbiterP$0$ResourceCmd$granted(
# 22 "/Users/jingyuancheng/tinyos/moteiv/tos/lib/resource/FcfsArbiterP.nc"
uint8_t arg_0x101b3b660, 
# 70 "/Users/jingyuancheng/tinyos/moteiv/tos/lib/resource/ResourceCmd.nc"
uint8_t rh);
# 29 "/Users/jingyuancheng/tinyos/moteiv/tos/lib/resource/ResourceConfigure.nc"
static void /*MSP430ArbiterTimerAC.ArbiterC.ArbiterP*/FcfsArbiterP$0$ResourceConfigure$configure(
# 24 "/Users/jingyuancheng/tinyos/moteiv/tos/lib/resource/FcfsArbiterP.nc"
uint8_t arg_0x101b719b0);
# 80 "/Users/jingyuancheng/tinyos/moteiv/tos/lib/resource/ResourceCmdAsync.nc"
static void /*MSP430ArbiterTimerAC.ArbiterC.ArbiterP*/FcfsArbiterP$0$ResourceCmdAsync$granted(
# 23 "/Users/jingyuancheng/tinyos/moteiv/tos/lib/resource/FcfsArbiterP.nc"
uint8_t arg_0x101b3a698, 
# 80 "/Users/jingyuancheng/tinyos/moteiv/tos/lib/resource/ResourceCmdAsync.nc"
uint8_t rh);
# 54 "/Users/jingyuancheng/tinyos/moteiv/tos/lib/resource/Arbiter.nc"
static void /*MSP430ArbiterTimerAC.ArbiterC.ArbiterP*/FcfsArbiterP$0$Arbiter$requested(void );





static void /*MSP430ArbiterTimerAC.ArbiterC.ArbiterP*/FcfsArbiterP$0$Arbiter$idle(void );
# 50 "/Users/jingyuancheng/tinyos/moteiv/tos/lib/sched/TaskBasic.nc"
static result_t /*MSP430ArbiterTimerAC.ArbiterC.ArbiterP*/FcfsArbiterP$0$GrantTask$postUrgentTask(void );
#line 49
static result_t /*MSP430ArbiterTimerAC.ArbiterC.ArbiterP*/FcfsArbiterP$0$GrantTask$postTask(void );
# 73 "/Users/jingyuancheng/tinyos/moteiv/tos/lib/resource/Resource.nc"
static void /*MSP430ArbiterTimerAC.ArbiterC.ArbiterP*/FcfsArbiterP$0$Resource$granted(
# 21 "/Users/jingyuancheng/tinyos/moteiv/tos/lib/resource/FcfsArbiterP.nc"
uint8_t arg_0x101b3c648);







enum /*MSP430ArbiterTimerAC.ArbiterC.ArbiterP*/FcfsArbiterP$0$__nesc_unnamed4334 {
  FcfsArbiterP$0$COUNT = 3U
};





#line 33
struct /*MSP430ArbiterTimerAC.ArbiterC.ArbiterP*/FcfsArbiterP$0$__nesc_unnamed4335 {
  uint8_t head;
  uint8_t tail;
  uint8_t next[/*MSP430ArbiterTimerAC.ArbiterC.ArbiterP*/FcfsArbiterP$0$COUNT];
} /*MSP430ArbiterTimerAC.ArbiterC.ArbiterP*/FcfsArbiterP$0$m_queue;

uint8_t /*MSP430ArbiterTimerAC.ArbiterC.ArbiterP*/FcfsArbiterP$0$m_urgentCount;
uint8_t /*MSP430ArbiterTimerAC.ArbiterC.ArbiterP*/FcfsArbiterP$0$m_granted;


static inline ReservedQueue_t */*MSP430ArbiterTimerAC.ArbiterC.ArbiterP*/FcfsArbiterP$0$queue(void );




static inline result_t /*MSP430ArbiterTimerAC.ArbiterC.ArbiterP*/FcfsArbiterP$0$Init$init(void );








static inline void /*MSP430ArbiterTimerAC.ArbiterC.ArbiterP*/FcfsArbiterP$0$GrantTask$runTask(void );
#line 76
static bool /*MSP430ArbiterTimerAC.ArbiterC.ArbiterP*/FcfsArbiterP$0$ResourceValidate$validateUser(uint8_t rh);



static inline void /*MSP430ArbiterTimerAC.ArbiterC.ArbiterP*/FcfsArbiterP$0$Resource$request(uint8_t id);
#line 105
static void /*MSP430ArbiterTimerAC.ArbiterC.ArbiterP*/FcfsArbiterP$0$Resource$release(uint8_t id);
#line 155
static inline void /*MSP430ArbiterTimerAC.ArbiterC.ArbiterP*/FcfsArbiterP$0$ResourceCmd$deferRequest(uint8_t id);



static inline void /*MSP430ArbiterTimerAC.ArbiterC.ArbiterP*/FcfsArbiterP$0$ResourceCmd$release(uint8_t id);
#line 204
static inline uint8_t /*MSP430ArbiterTimerAC.ArbiterC.ArbiterP*/FcfsArbiterP$0$ResourceCmdAsync$immediateRequest(uint8_t id, uint8_t rh);
#line 219
static inline void /*MSP430ArbiterTimerAC.ArbiterC.ArbiterP*/FcfsArbiterP$0$ResourceCmdAsync$release(uint8_t id);










static inline void /*MSP430ArbiterTimerAC.ArbiterC.ArbiterP*/FcfsArbiterP$0$Resource$default$granted(uint8_t id);


static inline void /*MSP430ArbiterTimerAC.ArbiterC.ArbiterP*/FcfsArbiterP$0$ResourceCmd$default$granted(uint8_t id, uint8_t rh);


static inline void /*MSP430ArbiterTimerAC.ArbiterC.ArbiterP*/FcfsArbiterP$0$ResourceCmdAsync$default$granted(uint8_t id, uint8_t rh);
# 58 "/Users/jingyuancheng/tinyos/moteiv/tos/lib/sched/TaskBasic.nc"
static void SchedulerBasicP$TaskBasic$runTask(
# 51 "/Users/jingyuancheng/tinyos/moteiv/tos/lib/sched/SchedulerBasicP.nc"
uint8_t arg_0x101550368);






enum SchedulerBasicP$__nesc_unnamed4336 {
  SchedulerBasicP$NUM_TASKS = 23U, 
  SchedulerBasicP$NONE = 255
};

uint8_t SchedulerBasicP$m_head;
uint8_t SchedulerBasicP$m_tail;
uint8_t SchedulerBasicP$m_next[SchedulerBasicP$NUM_TASKS];
#line 96
static inline bool SchedulerBasicP$isQueued(uint8_t id);






static inline uint8_t SchedulerBasicP$popTask(void );










static bool SchedulerBasicP$pushTask(uint8_t id);
#line 131
static bool SchedulerBasicP$pushFront(uint8_t id);
#line 145
static inline void SchedulerBasicP$Scheduler$init(void );









static bool SchedulerBasicP$Scheduler$runNextTask(bool sleep);
#line 191
static inline result_t SchedulerBasicP$TaskBasic$postTask(uint8_t id);




static inline result_t SchedulerBasicP$TaskBasic$postUrgentTask(uint8_t id);




static inline void SchedulerBasicP$TaskBasic$default$runTask(uint8_t id);
# 29 "/Users/jingyuancheng/tinyos/moteiv/tos/lib/resource/ResourceConfigure.nc"
static void MSP430ResourceConfigTimerAP$ConfigTimerA$configure(
# 17 "/Users/jingyuancheng/tinyos/moteiv/tos/platform/msp430/resource/MSP430ResourceConfigTimerAP.nc"
uint8_t arg_0x101bbdc98);
# 35 "/Users/jingyuancheng/tinyos/moteiv/tos/platform/msp430/timer/MSP430Timer.nc"
static void MSP430ResourceConfigTimerAP$TimerA$setMode(int mode);
# 23 "/Users/jingyuancheng/tinyos/moteiv/tos/platform/msp430/resource/MSP430ResourceConfigTimerAP.nc"
static void MSP430ResourceConfigTimerAP$idle(void );



static inline void MSP430ResourceConfigTimerAP$Arbiter$idle(void );



static inline void MSP430ResourceConfigTimerAP$Arbiter$requested(void );


static inline void MSP430ResourceConfigTimerAP$WrapConfigTimerA$configure(uint8_t rh);





static inline void MSP430ResourceConfigTimerAP$ConfigTimerA$default$configure(uint8_t rh);


static inline void MSP430ResourceConfigTimerAP$TimerA$overflow(void );
# 4 "/Users/jingyuancheng/tinyos/moteiv/tos/platform/msp430/timer/MSP430TimerEvent.nc"
static void /*MSP430TimerC.MSP430TimerA*/MSP430TimerM$0$Event$fired(
# 39 "/Users/jingyuancheng/tinyos/moteiv/tos/platform/msp430/timer/MSP430TimerM.nc"
uint8_t arg_0x101bf0748);
# 33 "/Users/jingyuancheng/tinyos/moteiv/tos/platform/msp430/timer/MSP430Timer.nc"
static void /*MSP430TimerC.MSP430TimerA*/MSP430TimerM$0$Timer$overflow(void );
# 50 "/Users/jingyuancheng/tinyos/moteiv/tos/platform/msp430/timer/MSP430TimerM.nc"
static inline uint16_t /*MSP430TimerC.MSP430TimerA*/MSP430TimerM$0$Timer$get(void );
#line 79
static void /*MSP430TimerC.MSP430TimerA*/MSP430TimerM$0$Timer$setMode(int mode);









static inline void /*MSP430TimerC.MSP430TimerA*/MSP430TimerM$0$Timer$clear(void );









static inline void /*MSP430TimerC.MSP430TimerA*/MSP430TimerM$0$Timer$disableEvents(void );




static void /*MSP430TimerC.MSP430TimerA*/MSP430TimerM$0$Timer$setClockSource(uint16_t clockSource);




static void /*MSP430TimerC.MSP430TimerA*/MSP430TimerM$0$Timer$setInputDivider(uint16_t inputDivider);




static inline void /*MSP430TimerC.MSP430TimerA*/MSP430TimerM$0$VectorTimerX0$fired(void );




static inline void /*MSP430TimerC.MSP430TimerA*/MSP430TimerM$0$VectorTimerX1$fired(void );





static inline void /*MSP430TimerC.MSP430TimerA*/MSP430TimerM$0$Overflow$fired(void );








static inline void /*MSP430TimerC.MSP430TimerA*/MSP430TimerM$0$Event$default$fired(uint8_t n);
# 4 "/Users/jingyuancheng/tinyos/moteiv/tos/platform/msp430/timer/MSP430TimerEvent.nc"
static void /*MSP430TimerC.MSP430TimerB*/MSP430TimerM$1$Event$fired(
# 39 "/Users/jingyuancheng/tinyos/moteiv/tos/platform/msp430/timer/MSP430TimerM.nc"
uint8_t arg_0x101bf0748);
# 33 "/Users/jingyuancheng/tinyos/moteiv/tos/platform/msp430/timer/MSP430Timer.nc"
static void /*MSP430TimerC.MSP430TimerB*/MSP430TimerM$1$Timer$overflow(void );
# 50 "/Users/jingyuancheng/tinyos/moteiv/tos/platform/msp430/timer/MSP430TimerM.nc"
static uint16_t /*MSP430TimerC.MSP430TimerB*/MSP430TimerM$1$Timer$get(void );
#line 69
static inline bool /*MSP430TimerC.MSP430TimerB*/MSP430TimerM$1$Timer$isOverflowPending(void );
#line 114
static inline void /*MSP430TimerC.MSP430TimerB*/MSP430TimerM$1$VectorTimerX0$fired(void );




static inline void /*MSP430TimerC.MSP430TimerB*/MSP430TimerM$1$VectorTimerX1$fired(void );





static inline void /*MSP430TimerC.MSP430TimerB*/MSP430TimerM$1$Overflow$fired(void );








static void /*MSP430TimerC.MSP430TimerB*/MSP430TimerM$1$Event$default$fired(uint8_t n);
# 74 "/Users/jingyuancheng/tinyos/moteiv/tos/platform/msp430/timer/MSP430Capture.nc"
static void /*MSP430TimerC.MSP430TimerA0*/MSP430TimerCapComM$0$Capture$captured(uint16_t time);
# 34 "/Users/jingyuancheng/tinyos/moteiv/tos/platform/msp430/timer/MSP430Compare.nc"
static void /*MSP430TimerC.MSP430TimerA0*/MSP430TimerCapComM$0$Compare$fired(void );
# 43 "/Users/jingyuancheng/tinyos/moteiv/tos/platform/msp430/timer/MSP430TimerCapComM.nc"
typedef MSP430CompareControl_t /*MSP430TimerC.MSP430TimerA0*/MSP430TimerCapComM$0$CC_t;

static inline uint16_t /*MSP430TimerC.MSP430TimerA0*/MSP430TimerCapComM$0$CC2int(/*MSP430TimerC.MSP430TimerA0*/MSP430TimerCapComM$0$CC_t x);
static inline /*MSP430TimerC.MSP430TimerA0*/MSP430TimerCapComM$0$CC_t /*MSP430TimerC.MSP430TimerA0*/MSP430TimerCapComM$0$int2CC(uint16_t x);
#line 73
static inline /*MSP430TimerC.MSP430TimerA0*/MSP430TimerCapComM$0$CC_t /*MSP430TimerC.MSP430TimerA0*/MSP430TimerCapComM$0$Control$getControl(void );
#line 88
static inline void /*MSP430TimerC.MSP430TimerA0*/MSP430TimerCapComM$0$Control$setControl(/*MSP430TimerC.MSP430TimerA0*/MSP430TimerCapComM$0$CC_t x);
#line 138
static inline uint16_t /*MSP430TimerC.MSP430TimerA0*/MSP430TimerCapComM$0$Capture$getEvent(void );




static inline void /*MSP430TimerC.MSP430TimerA0*/MSP430TimerCapComM$0$Compare$setEvent(uint16_t x);
#line 168
static void /*MSP430TimerC.MSP430TimerA0*/MSP430TimerCapComM$0$Event$fired(void );







static inline void /*MSP430TimerC.MSP430TimerA0*/MSP430TimerCapComM$0$Capture$default$captured(uint16_t n);







static inline void /*MSP430TimerC.MSP430TimerA0*/MSP430TimerCapComM$0$Timer$overflow(void );
# 74 "/Users/jingyuancheng/tinyos/moteiv/tos/platform/msp430/timer/MSP430Capture.nc"
static void /*MSP430TimerC.MSP430TimerA1*/MSP430TimerCapComM$1$Capture$captured(uint16_t time);
# 34 "/Users/jingyuancheng/tinyos/moteiv/tos/platform/msp430/timer/MSP430Compare.nc"
static void /*MSP430TimerC.MSP430TimerA1*/MSP430TimerCapComM$1$Compare$fired(void );
# 43 "/Users/jingyuancheng/tinyos/moteiv/tos/platform/msp430/timer/MSP430TimerCapComM.nc"
typedef MSP430CompareControl_t /*MSP430TimerC.MSP430TimerA1*/MSP430TimerCapComM$1$CC_t;

static inline uint16_t /*MSP430TimerC.MSP430TimerA1*/MSP430TimerCapComM$1$CC2int(/*MSP430TimerC.MSP430TimerA1*/MSP430TimerCapComM$1$CC_t x);
static inline /*MSP430TimerC.MSP430TimerA1*/MSP430TimerCapComM$1$CC_t /*MSP430TimerC.MSP430TimerA1*/MSP430TimerCapComM$1$int2CC(uint16_t x);
#line 73
static inline /*MSP430TimerC.MSP430TimerA1*/MSP430TimerCapComM$1$CC_t /*MSP430TimerC.MSP430TimerA1*/MSP430TimerCapComM$1$Control$getControl(void );
#line 88
static inline void /*MSP430TimerC.MSP430TimerA1*/MSP430TimerCapComM$1$Control$setControl(/*MSP430TimerC.MSP430TimerA1*/MSP430TimerCapComM$1$CC_t x);
#line 138
static inline uint16_t /*MSP430TimerC.MSP430TimerA1*/MSP430TimerCapComM$1$Capture$getEvent(void );




static inline void /*MSP430TimerC.MSP430TimerA1*/MSP430TimerCapComM$1$Compare$setEvent(uint16_t x);
#line 168
static void /*MSP430TimerC.MSP430TimerA1*/MSP430TimerCapComM$1$Event$fired(void );







static inline void /*MSP430TimerC.MSP430TimerA1*/MSP430TimerCapComM$1$Capture$default$captured(uint16_t n);







static inline void /*MSP430TimerC.MSP430TimerA1*/MSP430TimerCapComM$1$Timer$overflow(void );
# 74 "/Users/jingyuancheng/tinyos/moteiv/tos/platform/msp430/timer/MSP430Capture.nc"
static void /*MSP430TimerC.MSP430TimerA2*/MSP430TimerCapComM$2$Capture$captured(uint16_t time);
# 34 "/Users/jingyuancheng/tinyos/moteiv/tos/platform/msp430/timer/MSP430Compare.nc"
static void /*MSP430TimerC.MSP430TimerA2*/MSP430TimerCapComM$2$Compare$fired(void );
# 43 "/Users/jingyuancheng/tinyos/moteiv/tos/platform/msp430/timer/MSP430TimerCapComM.nc"
typedef MSP430CompareControl_t /*MSP430TimerC.MSP430TimerA2*/MSP430TimerCapComM$2$CC_t;


static inline /*MSP430TimerC.MSP430TimerA2*/MSP430TimerCapComM$2$CC_t /*MSP430TimerC.MSP430TimerA2*/MSP430TimerCapComM$2$int2CC(uint16_t x);
#line 73
static inline /*MSP430TimerC.MSP430TimerA2*/MSP430TimerCapComM$2$CC_t /*MSP430TimerC.MSP430TimerA2*/MSP430TimerCapComM$2$Control$getControl(void );
#line 138
static inline uint16_t /*MSP430TimerC.MSP430TimerA2*/MSP430TimerCapComM$2$Capture$getEvent(void );
#line 168
static void /*MSP430TimerC.MSP430TimerA2*/MSP430TimerCapComM$2$Event$fired(void );







static inline void /*MSP430TimerC.MSP430TimerA2*/MSP430TimerCapComM$2$Capture$default$captured(uint16_t n);



static inline void /*MSP430TimerC.MSP430TimerA2*/MSP430TimerCapComM$2$Compare$default$fired(void );



static inline void /*MSP430TimerC.MSP430TimerA2*/MSP430TimerCapComM$2$Timer$overflow(void );
# 74 "/Users/jingyuancheng/tinyos/moteiv/tos/platform/msp430/timer/MSP430Capture.nc"
static void /*MSP430TimerC.MSP430TimerB0*/MSP430TimerCapComM$3$Capture$captured(uint16_t time);
# 34 "/Users/jingyuancheng/tinyos/moteiv/tos/platform/msp430/timer/MSP430Compare.nc"
static void /*MSP430TimerC.MSP430TimerB0*/MSP430TimerCapComM$3$Compare$fired(void );
# 30 "/Users/jingyuancheng/tinyos/moteiv/tos/platform/msp430/timer/MSP430Timer.nc"
static uint16_t /*MSP430TimerC.MSP430TimerB0*/MSP430TimerCapComM$3$Timer$get(void );
# 43 "/Users/jingyuancheng/tinyos/moteiv/tos/platform/msp430/timer/MSP430TimerCapComM.nc"
typedef MSP430CompareControl_t /*MSP430TimerC.MSP430TimerB0*/MSP430TimerCapComM$3$CC_t;


static inline /*MSP430TimerC.MSP430TimerB0*/MSP430TimerCapComM$3$CC_t /*MSP430TimerC.MSP430TimerB0*/MSP430TimerCapComM$3$int2CC(uint16_t x);
#line 73
static inline /*MSP430TimerC.MSP430TimerB0*/MSP430TimerCapComM$3$CC_t /*MSP430TimerC.MSP430TimerB0*/MSP430TimerCapComM$3$Control$getControl(void );




static inline bool /*MSP430TimerC.MSP430TimerB0*/MSP430TimerCapComM$3$Control$isInterruptPending(void );




static inline void /*MSP430TimerC.MSP430TimerB0*/MSP430TimerCapComM$3$Control$clearPendingInterrupt(void );
#line 123
static inline void /*MSP430TimerC.MSP430TimerB0*/MSP430TimerCapComM$3$Control$disableEvents(void );
#line 138
static inline uint16_t /*MSP430TimerC.MSP430TimerB0*/MSP430TimerCapComM$3$Capture$getEvent(void );









static inline void /*MSP430TimerC.MSP430TimerB0*/MSP430TimerCapComM$3$Compare$setEventFromPrev(uint16_t x);




static inline void /*MSP430TimerC.MSP430TimerB0*/MSP430TimerCapComM$3$Compare$setEventFromNow(uint16_t x);
#line 168
static inline void /*MSP430TimerC.MSP430TimerB0*/MSP430TimerCapComM$3$Event$fired(void );







static inline void /*MSP430TimerC.MSP430TimerB0*/MSP430TimerCapComM$3$Capture$default$captured(uint16_t n);







static inline void /*MSP430TimerC.MSP430TimerB0*/MSP430TimerCapComM$3$Timer$overflow(void );
# 74 "/Users/jingyuancheng/tinyos/moteiv/tos/platform/msp430/timer/MSP430Capture.nc"
static void /*MSP430TimerC.MSP430TimerB1*/MSP430TimerCapComM$4$Capture$captured(uint16_t time);
# 34 "/Users/jingyuancheng/tinyos/moteiv/tos/platform/msp430/timer/MSP430Compare.nc"
static void /*MSP430TimerC.MSP430TimerB1*/MSP430TimerCapComM$4$Compare$fired(void );
# 43 "/Users/jingyuancheng/tinyos/moteiv/tos/platform/msp430/timer/MSP430TimerCapComM.nc"
typedef MSP430CompareControl_t /*MSP430TimerC.MSP430TimerB1*/MSP430TimerCapComM$4$CC_t;

static inline uint16_t /*MSP430TimerC.MSP430TimerB1*/MSP430TimerCapComM$4$CC2int(/*MSP430TimerC.MSP430TimerB1*/MSP430TimerCapComM$4$CC_t x);
static inline /*MSP430TimerC.MSP430TimerB1*/MSP430TimerCapComM$4$CC_t /*MSP430TimerC.MSP430TimerB1*/MSP430TimerCapComM$4$int2CC(uint16_t x);
#line 60
static inline uint16_t /*MSP430TimerC.MSP430TimerB1*/MSP430TimerCapComM$4$captureControl(uint8_t l_cm);
#line 73
static inline /*MSP430TimerC.MSP430TimerB1*/MSP430TimerCapComM$4$CC_t /*MSP430TimerC.MSP430TimerB1*/MSP430TimerCapComM$4$Control$getControl(void );









static inline void /*MSP430TimerC.MSP430TimerB1*/MSP430TimerCapComM$4$Control$clearPendingInterrupt(void );
#line 98
static inline void /*MSP430TimerC.MSP430TimerB1*/MSP430TimerCapComM$4$Control$setControlAsCapture(uint8_t capture_mode);
#line 118
static inline void /*MSP430TimerC.MSP430TimerB1*/MSP430TimerCapComM$4$Control$enableEvents(void );




static inline void /*MSP430TimerC.MSP430TimerB1*/MSP430TimerCapComM$4$Control$disableEvents(void );
#line 138
static inline uint16_t /*MSP430TimerC.MSP430TimerB1*/MSP430TimerCapComM$4$Capture$getEvent(void );
#line 158
static inline bool /*MSP430TimerC.MSP430TimerB1*/MSP430TimerCapComM$4$Capture$isOverflowPending(void );




static inline void /*MSP430TimerC.MSP430TimerB1*/MSP430TimerCapComM$4$Capture$clearOverflow(void );




static inline void /*MSP430TimerC.MSP430TimerB1*/MSP430TimerCapComM$4$Event$fired(void );
#line 180
static inline void /*MSP430TimerC.MSP430TimerB1*/MSP430TimerCapComM$4$Compare$default$fired(void );



static inline void /*MSP430TimerC.MSP430TimerB1*/MSP430TimerCapComM$4$Timer$overflow(void );
# 74 "/Users/jingyuancheng/tinyos/moteiv/tos/platform/msp430/timer/MSP430Capture.nc"
static void /*MSP430TimerC.MSP430TimerB2*/MSP430TimerCapComM$5$Capture$captured(uint16_t time);
# 34 "/Users/jingyuancheng/tinyos/moteiv/tos/platform/msp430/timer/MSP430Compare.nc"
static void /*MSP430TimerC.MSP430TimerB2*/MSP430TimerCapComM$5$Compare$fired(void );
# 30 "/Users/jingyuancheng/tinyos/moteiv/tos/platform/msp430/timer/MSP430Timer.nc"
static uint16_t /*MSP430TimerC.MSP430TimerB2*/MSP430TimerCapComM$5$Timer$get(void );
# 43 "/Users/jingyuancheng/tinyos/moteiv/tos/platform/msp430/timer/MSP430TimerCapComM.nc"
typedef MSP430CompareControl_t /*MSP430TimerC.MSP430TimerB2*/MSP430TimerCapComM$5$CC_t;

static inline uint16_t /*MSP430TimerC.MSP430TimerB2*/MSP430TimerCapComM$5$CC2int(/*MSP430TimerC.MSP430TimerB2*/MSP430TimerCapComM$5$CC_t x);
static inline /*MSP430TimerC.MSP430TimerB2*/MSP430TimerCapComM$5$CC_t /*MSP430TimerC.MSP430TimerB2*/MSP430TimerCapComM$5$int2CC(uint16_t x);

static inline uint16_t /*MSP430TimerC.MSP430TimerB2*/MSP430TimerCapComM$5$compareControl(void );
#line 73
static inline /*MSP430TimerC.MSP430TimerB2*/MSP430TimerCapComM$5$CC_t /*MSP430TimerC.MSP430TimerB2*/MSP430TimerCapComM$5$Control$getControl(void );









static inline void /*MSP430TimerC.MSP430TimerB2*/MSP430TimerCapComM$5$Control$clearPendingInterrupt(void );









static inline void /*MSP430TimerC.MSP430TimerB2*/MSP430TimerCapComM$5$Control$setControlAsCompare(void );
#line 118
static inline void /*MSP430TimerC.MSP430TimerB2*/MSP430TimerCapComM$5$Control$enableEvents(void );




static inline void /*MSP430TimerC.MSP430TimerB2*/MSP430TimerCapComM$5$Control$disableEvents(void );
#line 138
static inline uint16_t /*MSP430TimerC.MSP430TimerB2*/MSP430TimerCapComM$5$Capture$getEvent(void );




static inline void /*MSP430TimerC.MSP430TimerB2*/MSP430TimerCapComM$5$Compare$setEvent(uint16_t x);









static inline void /*MSP430TimerC.MSP430TimerB2*/MSP430TimerCapComM$5$Compare$setEventFromNow(uint16_t x);
#line 168
static inline void /*MSP430TimerC.MSP430TimerB2*/MSP430TimerCapComM$5$Event$fired(void );







static inline void /*MSP430TimerC.MSP430TimerB2*/MSP430TimerCapComM$5$Capture$default$captured(uint16_t n);







static inline void /*MSP430TimerC.MSP430TimerB2*/MSP430TimerCapComM$5$Timer$overflow(void );
# 74 "/Users/jingyuancheng/tinyos/moteiv/tos/platform/msp430/timer/MSP430Capture.nc"
static void /*MSP430TimerC.MSP430TimerB3*/MSP430TimerCapComM$6$Capture$captured(uint16_t time);
# 34 "/Users/jingyuancheng/tinyos/moteiv/tos/platform/msp430/timer/MSP430Compare.nc"
static void /*MSP430TimerC.MSP430TimerB3*/MSP430TimerCapComM$6$Compare$fired(void );
# 30 "/Users/jingyuancheng/tinyos/moteiv/tos/platform/msp430/timer/MSP430Timer.nc"
static uint16_t /*MSP430TimerC.MSP430TimerB3*/MSP430TimerCapComM$6$Timer$get(void );
# 43 "/Users/jingyuancheng/tinyos/moteiv/tos/platform/msp430/timer/MSP430TimerCapComM.nc"
typedef MSP430CompareControl_t /*MSP430TimerC.MSP430TimerB3*/MSP430TimerCapComM$6$CC_t;

static inline uint16_t /*MSP430TimerC.MSP430TimerB3*/MSP430TimerCapComM$6$CC2int(/*MSP430TimerC.MSP430TimerB3*/MSP430TimerCapComM$6$CC_t x);
static inline /*MSP430TimerC.MSP430TimerB3*/MSP430TimerCapComM$6$CC_t /*MSP430TimerC.MSP430TimerB3*/MSP430TimerCapComM$6$int2CC(uint16_t x);

static inline uint16_t /*MSP430TimerC.MSP430TimerB3*/MSP430TimerCapComM$6$compareControl(void );
#line 73
static inline /*MSP430TimerC.MSP430TimerB3*/MSP430TimerCapComM$6$CC_t /*MSP430TimerC.MSP430TimerB3*/MSP430TimerCapComM$6$Control$getControl(void );









static inline void /*MSP430TimerC.MSP430TimerB3*/MSP430TimerCapComM$6$Control$clearPendingInterrupt(void );









static inline void /*MSP430TimerC.MSP430TimerB3*/MSP430TimerCapComM$6$Control$setControlAsCompare(void );
#line 118
static inline void /*MSP430TimerC.MSP430TimerB3*/MSP430TimerCapComM$6$Control$enableEvents(void );




static inline void /*MSP430TimerC.MSP430TimerB3*/MSP430TimerCapComM$6$Control$disableEvents(void );




static inline bool /*MSP430TimerC.MSP430TimerB3*/MSP430TimerCapComM$6$Control$areEventsEnabled(void );









static inline uint16_t /*MSP430TimerC.MSP430TimerB3*/MSP430TimerCapComM$6$Capture$getEvent(void );




static inline void /*MSP430TimerC.MSP430TimerB3*/MSP430TimerCapComM$6$Compare$setEvent(uint16_t x);









static inline void /*MSP430TimerC.MSP430TimerB3*/MSP430TimerCapComM$6$Compare$setEventFromNow(uint16_t x);
#line 168
static inline void /*MSP430TimerC.MSP430TimerB3*/MSP430TimerCapComM$6$Event$fired(void );







static inline void /*MSP430TimerC.MSP430TimerB3*/MSP430TimerCapComM$6$Capture$default$captured(uint16_t n);







static inline void /*MSP430TimerC.MSP430TimerB3*/MSP430TimerCapComM$6$Timer$overflow(void );
# 74 "/Users/jingyuancheng/tinyos/moteiv/tos/platform/msp430/timer/MSP430Capture.nc"
static void /*MSP430TimerC.MSP430TimerB4*/MSP430TimerCapComM$7$Capture$captured(uint16_t time);
# 34 "/Users/jingyuancheng/tinyos/moteiv/tos/platform/msp430/timer/MSP430Compare.nc"
static void /*MSP430TimerC.MSP430TimerB4*/MSP430TimerCapComM$7$Compare$fired(void );
# 30 "/Users/jingyuancheng/tinyos/moteiv/tos/platform/msp430/timer/MSP430Timer.nc"
static uint16_t /*MSP430TimerC.MSP430TimerB4*/MSP430TimerCapComM$7$Timer$get(void );
# 43 "/Users/jingyuancheng/tinyos/moteiv/tos/platform/msp430/timer/MSP430TimerCapComM.nc"
typedef MSP430CompareControl_t /*MSP430TimerC.MSP430TimerB4*/MSP430TimerCapComM$7$CC_t;

static inline uint16_t /*MSP430TimerC.MSP430TimerB4*/MSP430TimerCapComM$7$CC2int(/*MSP430TimerC.MSP430TimerB4*/MSP430TimerCapComM$7$CC_t x);
static inline /*MSP430TimerC.MSP430TimerB4*/MSP430TimerCapComM$7$CC_t /*MSP430TimerC.MSP430TimerB4*/MSP430TimerCapComM$7$int2CC(uint16_t x);

static inline uint16_t /*MSP430TimerC.MSP430TimerB4*/MSP430TimerCapComM$7$compareControl(void );
#line 73
static inline /*MSP430TimerC.MSP430TimerB4*/MSP430TimerCapComM$7$CC_t /*MSP430TimerC.MSP430TimerB4*/MSP430TimerCapComM$7$Control$getControl(void );









static inline void /*MSP430TimerC.MSP430TimerB4*/MSP430TimerCapComM$7$Control$clearPendingInterrupt(void );









static inline void /*MSP430TimerC.MSP430TimerB4*/MSP430TimerCapComM$7$Control$setControlAsCompare(void );
#line 118
static inline void /*MSP430TimerC.MSP430TimerB4*/MSP430TimerCapComM$7$Control$enableEvents(void );




static inline void /*MSP430TimerC.MSP430TimerB4*/MSP430TimerCapComM$7$Control$disableEvents(void );
#line 138
static inline uint16_t /*MSP430TimerC.MSP430TimerB4*/MSP430TimerCapComM$7$Capture$getEvent(void );




static inline void /*MSP430TimerC.MSP430TimerB4*/MSP430TimerCapComM$7$Compare$setEvent(uint16_t x);









static inline void /*MSP430TimerC.MSP430TimerB4*/MSP430TimerCapComM$7$Compare$setEventFromNow(uint16_t x);
#line 168
static inline void /*MSP430TimerC.MSP430TimerB4*/MSP430TimerCapComM$7$Event$fired(void );







static inline void /*MSP430TimerC.MSP430TimerB4*/MSP430TimerCapComM$7$Capture$default$captured(uint16_t n);







static inline void /*MSP430TimerC.MSP430TimerB4*/MSP430TimerCapComM$7$Timer$overflow(void );
# 74 "/Users/jingyuancheng/tinyos/moteiv/tos/platform/msp430/timer/MSP430Capture.nc"
static void /*MSP430TimerC.MSP430TimerB5*/MSP430TimerCapComM$8$Capture$captured(uint16_t time);
# 34 "/Users/jingyuancheng/tinyos/moteiv/tos/platform/msp430/timer/MSP430Compare.nc"
static void /*MSP430TimerC.MSP430TimerB5*/MSP430TimerCapComM$8$Compare$fired(void );
# 43 "/Users/jingyuancheng/tinyos/moteiv/tos/platform/msp430/timer/MSP430TimerCapComM.nc"
typedef MSP430CompareControl_t /*MSP430TimerC.MSP430TimerB5*/MSP430TimerCapComM$8$CC_t;


static inline /*MSP430TimerC.MSP430TimerB5*/MSP430TimerCapComM$8$CC_t /*MSP430TimerC.MSP430TimerB5*/MSP430TimerCapComM$8$int2CC(uint16_t x);
#line 73
static inline /*MSP430TimerC.MSP430TimerB5*/MSP430TimerCapComM$8$CC_t /*MSP430TimerC.MSP430TimerB5*/MSP430TimerCapComM$8$Control$getControl(void );
#line 138
static inline uint16_t /*MSP430TimerC.MSP430TimerB5*/MSP430TimerCapComM$8$Capture$getEvent(void );
#line 168
static inline void /*MSP430TimerC.MSP430TimerB5*/MSP430TimerCapComM$8$Event$fired(void );







static inline void /*MSP430TimerC.MSP430TimerB5*/MSP430TimerCapComM$8$Capture$default$captured(uint16_t n);



static inline void /*MSP430TimerC.MSP430TimerB5*/MSP430TimerCapComM$8$Compare$default$fired(void );



static inline void /*MSP430TimerC.MSP430TimerB5*/MSP430TimerCapComM$8$Timer$overflow(void );
# 74 "/Users/jingyuancheng/tinyos/moteiv/tos/platform/msp430/timer/MSP430Capture.nc"
static void /*MSP430TimerC.MSP430TimerB6*/MSP430TimerCapComM$9$Capture$captured(uint16_t time);
# 34 "/Users/jingyuancheng/tinyos/moteiv/tos/platform/msp430/timer/MSP430Compare.nc"
static void /*MSP430TimerC.MSP430TimerB6*/MSP430TimerCapComM$9$Compare$fired(void );
# 43 "/Users/jingyuancheng/tinyos/moteiv/tos/platform/msp430/timer/MSP430TimerCapComM.nc"
typedef MSP430CompareControl_t /*MSP430TimerC.MSP430TimerB6*/MSP430TimerCapComM$9$CC_t;


static inline /*MSP430TimerC.MSP430TimerB6*/MSP430TimerCapComM$9$CC_t /*MSP430TimerC.MSP430TimerB6*/MSP430TimerCapComM$9$int2CC(uint16_t x);
#line 73
static inline /*MSP430TimerC.MSP430TimerB6*/MSP430TimerCapComM$9$CC_t /*MSP430TimerC.MSP430TimerB6*/MSP430TimerCapComM$9$Control$getControl(void );
#line 138
static inline uint16_t /*MSP430TimerC.MSP430TimerB6*/MSP430TimerCapComM$9$Capture$getEvent(void );
#line 168
static inline void /*MSP430TimerC.MSP430TimerB6*/MSP430TimerCapComM$9$Event$fired(void );







static inline void /*MSP430TimerC.MSP430TimerB6*/MSP430TimerCapComM$9$Capture$default$captured(uint16_t n);



static inline void /*MSP430TimerC.MSP430TimerB6*/MSP430TimerCapComM$9$Compare$default$fired(void );



static inline void /*MSP430TimerC.MSP430TimerB6*/MSP430TimerCapComM$9$Timer$overflow(void );
# 4 "/Users/jingyuancheng/tinyos/moteiv/tos/platform/msp430/timer/MSP430TimerEvent.nc"
static void MSP430TimerCommonM$VectorTimerB1$fired(void );
#line 4
static void MSP430TimerCommonM$VectorTimerA0$fired(void );
#line 4
static void MSP430TimerCommonM$VectorTimerA1$fired(void );
#line 4
static void MSP430TimerCommonM$VectorTimerB0$fired(void );
# 11 "/Users/jingyuancheng/tinyos/moteiv/tos/platform/msp430/timer/MSP430TimerCommonM.nc"
void sig_TIMERA0_VECTOR(void )  __attribute((wakeup)) __attribute((interrupt(12))) ;
void sig_TIMERA1_VECTOR(void )  __attribute((wakeup)) __attribute((interrupt(10))) ;
void sig_TIMERB0_VECTOR(void )  __attribute((wakeup)) __attribute((interrupt(26))) ;
void sig_TIMERB1_VECTOR(void )  __attribute((wakeup)) __attribute((interrupt(24))) ;
# 40 "/Users/jingyuancheng/tinyos/moteiv/tos/lib/sched/Boot.nc"
static void MainP$Boot$booted(void );
# 63 "/Users/jingyuancheng/tinyos/moteiv/tinyos-1.x/tos/interfaces/StdControl.nc"
static result_t MainP$MainStdControl$init(
# 12 "/Users/jingyuancheng/tinyos/moteiv/tos/lib/sched/MainP.nc"
uint8_t arg_0x101d29020);
# 70 "/Users/jingyuancheng/tinyos/moteiv/tinyos-1.x/tos/interfaces/StdControl.nc"
static result_t MainP$MainStdControl$start(
# 12 "/Users/jingyuancheng/tinyos/moteiv/tos/lib/sched/MainP.nc"
uint8_t arg_0x101d29020);
# 64 "/Users/jingyuancheng/tinyos/moteiv/tinyos-1.x/tos/interfaces/SplitControl.nc"
static result_t MainP$MainSplitControl$init(
# 11 "/Users/jingyuancheng/tinyos/moteiv/tos/lib/sched/MainP.nc"
uint8_t arg_0x101d2caa8);
# 77 "/Users/jingyuancheng/tinyos/moteiv/tinyos-1.x/tos/interfaces/SplitControl.nc"
static result_t MainP$MainSplitControl$start(
# 11 "/Users/jingyuancheng/tinyos/moteiv/tos/lib/sched/MainP.nc"
uint8_t arg_0x101d2caa8);
# 46 "/Users/jingyuancheng/tinyos/moteiv/tos/lib/sched/Init.nc"
static result_t MainP$MainInit$init(
# 13 "/Users/jingyuancheng/tinyos/moteiv/tos/lib/sched/MainP.nc"
uint8_t arg_0x101d28020);
# 46 "/Users/jingyuancheng/tinyos/moteiv/tos/lib/sched/Init.nc"
static result_t MainP$PlatformInit$init(void );
# 41 "/Users/jingyuancheng/tinyos/moteiv/tos/lib/sched/Scheduler.nc"
static void MainP$Scheduler$init(void );








static bool MainP$Scheduler$runNextTask(bool l_sleep);
# 63 "/Users/jingyuancheng/tinyos/moteiv/tinyos-1.x/tos/interfaces/StdControl.nc"
static result_t MainP$StdControl$init(void );






static result_t MainP$StdControl$start(void );
# 17 "/Users/jingyuancheng/tinyos/moteiv/tos/lib/sched/MainP.nc"
uint8_t MainP$m_starting;

static inline void MainP$init(void );







static inline void MainP$startDone(uint8_t started);
#line 43
int main(void )   ;
#line 65
static inline result_t MainP$MainSplitControl$default$init(uint8_t id);




static inline result_t MainP$MainSplitControl$default$start(uint8_t id);










static inline result_t MainP$MainStdControl$default$init(uint8_t id);



static inline result_t MainP$MainStdControl$default$start(uint8_t id);








static inline result_t MainP$MainInit$default$init(uint8_t id);
#line 127
static inline void MainP$Boot$default$booted(void );
# 33 "/Users/jingyuancheng/tinyos/moteiv/tinyos-1.x/tos/platform/msp430/MSP430GeneralIO.nc"
static void ADC8IO14P$Ctrl7$makeOutput(void );
#line 27
static void ADC8IO14P$Ctrl7$setHigh(void );
static void ADC8IO14P$Ctrl7$setLow(void );
# 67 "/Users/jingyuancheng/tinyos/moteiv/tos/platform/msp430/adc/MSP430ADC12Single.nc"
static msp430ADCresult_t ADC8IO14P$ADC6$getData(void );
#line 52
static result_t ADC8IO14P$ADC6$bind(MSP430ADC12Settings_t settings);
#line 67
static msp430ADCresult_t ADC8IO14P$ADC0$getData(void );
#line 52
static result_t ADC8IO14P$ADC0$bind(MSP430ADC12Settings_t settings);
# 33 "/Users/jingyuancheng/tinyos/moteiv/tinyos-1.x/tos/platform/msp430/MSP430GeneralIO.nc"
static void ADC8IO14P$Ctrl5$makeOutput(void );
#line 27
static void ADC8IO14P$Ctrl5$setHigh(void );
static void ADC8IO14P$Ctrl5$setLow(void );




static void ADC8IO14P$Ctrl0$makeOutput(void );
#line 27
static void ADC8IO14P$Ctrl0$setHigh(void );
static void ADC8IO14P$Ctrl0$setLow(void );
# 67 "/Users/jingyuancheng/tinyos/moteiv/tos/platform/msp430/adc/MSP430ADC12Single.nc"
static msp430ADCresult_t ADC8IO14P$ADC3$getData(void );
#line 52
static result_t ADC8IO14P$ADC3$bind(MSP430ADC12Settings_t settings);
#line 67
static msp430ADCresult_t ADC8IO14P$ADC4$getData(void );
#line 52
static result_t ADC8IO14P$ADC4$bind(MSP430ADC12Settings_t settings);
# 33 "/Users/jingyuancheng/tinyos/moteiv/tinyos-1.x/tos/platform/msp430/MSP430GeneralIO.nc"
static void ADC8IO14P$Ctrl3$makeOutput(void );
#line 27
static void ADC8IO14P$Ctrl3$setHigh(void );
static void ADC8IO14P$Ctrl3$setLow(void );
# 67 "/Users/jingyuancheng/tinyos/moteiv/tos/platform/msp430/adc/MSP430ADC12Single.nc"
static msp430ADCresult_t ADC8IO14P$ADC7$getData(void );
#line 52
static result_t ADC8IO14P$ADC7$bind(MSP430ADC12Settings_t settings);
#line 67
static msp430ADCresult_t ADC8IO14P$ADC1$getData(void );
#line 52
static result_t ADC8IO14P$ADC1$bind(MSP430ADC12Settings_t settings);
# 33 "/Users/jingyuancheng/tinyos/moteiv/tinyos-1.x/tos/platform/msp430/MSP430GeneralIO.nc"
static void ADC8IO14P$LedUsr$makeOutput(void );
#line 27
static void ADC8IO14P$LedUsr$setHigh(void );
static void ADC8IO14P$LedUsr$setLow(void );




static void ADC8IO14P$Ctrl1$makeOutput(void );
#line 27
static void ADC8IO14P$Ctrl1$setHigh(void );
static void ADC8IO14P$Ctrl1$setLow(void );




static void ADC8IO14P$Ctrl6$makeOutput(void );
#line 27
static void ADC8IO14P$Ctrl6$setHigh(void );
static void ADC8IO14P$Ctrl6$setLow(void );
# 67 "/Users/jingyuancheng/tinyos/moteiv/tos/platform/msp430/adc/MSP430ADC12Single.nc"
static msp430ADCresult_t ADC8IO14P$ADC5$getData(void );
#line 52
static result_t ADC8IO14P$ADC5$bind(MSP430ADC12Settings_t settings);
# 33 "/Users/jingyuancheng/tinyos/moteiv/tinyos-1.x/tos/platform/msp430/MSP430GeneralIO.nc"
static void ADC8IO14P$Ctrl4$makeOutput(void );
#line 27
static void ADC8IO14P$Ctrl4$setHigh(void );
static void ADC8IO14P$Ctrl4$setLow(void );
# 48 "/Users/jingyuancheng/tinyos/moteiv/tinyos-1.x/tos/interfaces/SendMsg.nc"
static result_t ADC8IO14P$SendMsg$send(uint16_t address, uint8_t length, TOS_MsgPtr msg);
# 67 "/Users/jingyuancheng/tinyos/moteiv/tos/platform/msp430/adc/MSP430ADC12Single.nc"
static msp430ADCresult_t ADC8IO14P$ADC2$getData(void );
#line 52
static result_t ADC8IO14P$ADC2$bind(MSP430ADC12Settings_t settings);
# 49 "/Users/jingyuancheng/tinyos/moteiv/tos/lib/sched/TaskBasic.nc"
static result_t ADC8IO14P$taskSendData$postTask(void );
# 52 "/Users/jingyuancheng/tinyos/moteiv/tos/lib/timer/Timer2.nc"
static void ADC8IO14P$Timer$startPeriodic(uint32_t dt);
# 33 "/Users/jingyuancheng/tinyos/moteiv/tinyos-1.x/tos/platform/msp430/MSP430GeneralIO.nc"
static void ADC8IO14P$Ctrl2$makeOutput(void );
#line 27
static void ADC8IO14P$Ctrl2$setHigh(void );
static void ADC8IO14P$Ctrl2$setLow(void );
# 219 "ADC8IO14P.nc"
enum ADC8IO14P$__nesc_unnamed4337 {
#line 219
  ADC8IO14P$taskSendData = 1U
};
#line 219
typedef int ADC8IO14P$__nesc_sillytask_taskSendData[ADC8IO14P$taskSendData];
#line 45
TOS_Msg ADC8IO14P$m_msg;
uint16_t ADC8IO14P$m_count;
bool ADC8IO14P$m_sending;
uint8_t ADC8IO14P$countCtrl;

static inline result_t ADC8IO14P$StdControl$init(void );
#line 70
static inline result_t ADC8IO14P$StdControl$start(void );
#line 161
static void ADC8IO14P$switchCtrl(void );
#line 209
static inline void ADC8IO14P$Timer$fired(void );









static inline void ADC8IO14P$taskSendData$runTask(void );
#line 231
static inline result_t ADC8IO14P$ADC0$dataReady(uint16_t data);
#line 243
static inline result_t ADC8IO14P$ADC1$dataReady(uint16_t data);










static inline result_t ADC8IO14P$ADC2$dataReady(uint16_t data);









static inline result_t ADC8IO14P$ADC3$dataReady(uint16_t data);









static inline result_t ADC8IO14P$ADC4$dataReady(uint16_t data);









static inline result_t ADC8IO14P$ADC5$dataReady(uint16_t data);









static inline result_t ADC8IO14P$ADC6$dataReady(uint16_t data);










static inline result_t ADC8IO14P$ADC7$dataReady(uint16_t data);
#line 329
static inline result_t ADC8IO14P$SendMsg$sendDone(TOS_MsgPtr msg, result_t success);
# 30 "/Users/jingyuancheng/tinyos/moteiv/tos/platform/msp430/timer/MSP430Timer.nc"
static uint16_t /*HalTimerMilliC.AlarmMilliC.MSP430Alarm*/MSP430AlarmC$0$MSP430Timer$get(void );
# 30 "/Users/jingyuancheng/tinyos/moteiv/tos/platform/msp430/timer/MSP430Compare.nc"
static void /*HalTimerMilliC.AlarmMilliC.MSP430Alarm*/MSP430AlarmC$0$MSP430Compare$setEvent(uint16_t time);

static void /*HalTimerMilliC.AlarmMilliC.MSP430Alarm*/MSP430AlarmC$0$MSP430Compare$setEventFromNow(uint16_t delta);
# 64 "/Users/jingyuancheng/tinyos/moteiv/tos/lib/timer/Alarm.nc"
static void /*HalTimerMilliC.AlarmMilliC.MSP430Alarm*/MSP430AlarmC$0$Alarm$fired(void );
# 38 "/Users/jingyuancheng/tinyos/moteiv/tos/platform/msp430/timer/MSP430TimerControl.nc"
static void /*HalTimerMilliC.AlarmMilliC.MSP430Alarm*/MSP430AlarmC$0$MSP430TimerControl$enableEvents(void );
#line 35
static void /*HalTimerMilliC.AlarmMilliC.MSP430Alarm*/MSP430AlarmC$0$MSP430TimerControl$setControlAsCompare(void );



static void /*HalTimerMilliC.AlarmMilliC.MSP430Alarm*/MSP430AlarmC$0$MSP430TimerControl$disableEvents(void );
#line 32
static void /*HalTimerMilliC.AlarmMilliC.MSP430Alarm*/MSP430AlarmC$0$MSP430TimerControl$clearPendingInterrupt(void );
# 39 "/Users/jingyuancheng/tinyos/moteiv/tos/platform/msp430/timer/MSP430AlarmC.nc"
typedef result_t /*HalTimerMilliC.AlarmMilliC.MSP430Alarm*/MSP430AlarmC$0$error_t;

static inline /*HalTimerMilliC.AlarmMilliC.MSP430Alarm*/MSP430AlarmC$0$error_t /*HalTimerMilliC.AlarmMilliC.MSP430Alarm*/MSP430AlarmC$0$Init$init(void );






static inline result_t /*HalTimerMilliC.AlarmMilliC.MSP430Alarm*/MSP430AlarmC$0$Init$start(void );







static inline void /*HalTimerMilliC.AlarmMilliC.MSP430Alarm*/MSP430AlarmC$0$Alarm$stop(void );




static inline void /*HalTimerMilliC.AlarmMilliC.MSP430Alarm*/MSP430AlarmC$0$MSP430Compare$fired(void );










static void /*HalTimerMilliC.AlarmMilliC.MSP430Alarm*/MSP430AlarmC$0$Alarm$startAt(uint16_t t0, uint16_t dt);
#line 105
static inline void /*HalTimerMilliC.AlarmMilliC.MSP430Alarm*/MSP430AlarmC$0$MSP430Timer$overflow(void );
# 64 "/Users/jingyuancheng/tinyos/moteiv/tos/lib/timer/Alarm.nc"
static void /*HalTimerMilliC.AlarmMilliC.Transform*/TransformAlarmC$0$Alarm$fired(void );
#line 88
static void /*HalTimerMilliC.AlarmMilliC.Transform*/TransformAlarmC$0$AlarmFrom$startAt(/*HalTimerMilliC.AlarmMilliC.Transform*/TransformAlarmC$0$AlarmFrom$size_type t0, /*HalTimerMilliC.AlarmMilliC.Transform*/TransformAlarmC$0$AlarmFrom$size_type dt);
#line 60
static void /*HalTimerMilliC.AlarmMilliC.Transform*/TransformAlarmC$0$AlarmFrom$stop(void );
# 52 "/Users/jingyuancheng/tinyos/moteiv/tos/lib/timer/Counter.nc"
static /*HalTimerMilliC.AlarmMilliC.Transform*/TransformAlarmC$0$Counter$size_type /*HalTimerMilliC.AlarmMilliC.Transform*/TransformAlarmC$0$Counter$get(void );
# 40 "/Users/jingyuancheng/tinyos/moteiv/tos/lib/timer/TransformAlarmC.nc"
/*HalTimerMilliC.AlarmMilliC.Transform*/TransformAlarmC$0$to_size_type /*HalTimerMilliC.AlarmMilliC.Transform*/TransformAlarmC$0$m_t0;
/*HalTimerMilliC.AlarmMilliC.Transform*/TransformAlarmC$0$to_size_type /*HalTimerMilliC.AlarmMilliC.Transform*/TransformAlarmC$0$m_dt;

enum /*HalTimerMilliC.AlarmMilliC.Transform*/TransformAlarmC$0$__nesc_unnamed4338 {

  TransformAlarmC$0$MAX_DELAY_LOG2 = 8 * sizeof(/*HalTimerMilliC.AlarmMilliC.Transform*/TransformAlarmC$0$from_size_type ) - 1 - 5, 
  TransformAlarmC$0$MAX_DELAY = (/*HalTimerMilliC.AlarmMilliC.Transform*/TransformAlarmC$0$to_size_type )1 << /*HalTimerMilliC.AlarmMilliC.Transform*/TransformAlarmC$0$MAX_DELAY_LOG2
};

static inline /*HalTimerMilliC.AlarmMilliC.Transform*/TransformAlarmC$0$to_size_type /*HalTimerMilliC.AlarmMilliC.Transform*/TransformAlarmC$0$Alarm$getNow(void );
#line 65
static inline void /*HalTimerMilliC.AlarmMilliC.Transform*/TransformAlarmC$0$Alarm$stop(void );




static void /*HalTimerMilliC.AlarmMilliC.Transform*/TransformAlarmC$0$set_alarm(void );
#line 100
static void /*HalTimerMilliC.AlarmMilliC.Transform*/TransformAlarmC$0$Alarm$startAt(/*HalTimerMilliC.AlarmMilliC.Transform*/TransformAlarmC$0$to_size_type t0, /*HalTimerMilliC.AlarmMilliC.Transform*/TransformAlarmC$0$to_size_type dt);
#line 115
static inline void /*HalTimerMilliC.AlarmMilliC.Transform*/TransformAlarmC$0$AlarmFrom$fired(void );
#line 130
static inline void /*HalTimerMilliC.AlarmMilliC.Transform*/TransformAlarmC$0$Counter$overflow(void );
# 30 "/Users/jingyuancheng/tinyos/moteiv/tos/platform/msp430/timer/MSP430Timer.nc"
static uint16_t /*MSP430Counter32khzC.Counter*/MSP430CounterC$0$MSP430Timer$get(void );
static bool /*MSP430Counter32khzC.Counter*/MSP430CounterC$0$MSP430Timer$isOverflowPending(void );
# 70 "/Users/jingyuancheng/tinyos/moteiv/tos/lib/timer/Counter.nc"
static void /*MSP430Counter32khzC.Counter*/MSP430CounterC$0$Counter$overflow(void );
# 36 "/Users/jingyuancheng/tinyos/moteiv/tos/platform/msp430/timer/MSP430CounterC.nc"
static inline uint16_t /*MSP430Counter32khzC.Counter*/MSP430CounterC$0$Counter$get(void );




static inline bool /*MSP430Counter32khzC.Counter*/MSP430CounterC$0$Counter$isOverflowPending(void );









static inline void /*MSP430Counter32khzC.Counter*/MSP430CounterC$0$MSP430Timer$overflow(void );
# 52 "/Users/jingyuancheng/tinyos/moteiv/tos/lib/timer/Counter.nc"
static /*CounterMilliC.Transform*/TransformCounterC$0$CounterFrom$size_type /*CounterMilliC.Transform*/TransformCounterC$0$CounterFrom$get(void );






static bool /*CounterMilliC.Transform*/TransformCounterC$0$CounterFrom$isOverflowPending(void );










static void /*CounterMilliC.Transform*/TransformCounterC$0$Counter$overflow(void );
# 46 "/Users/jingyuancheng/tinyos/moteiv/tos/lib/timer/TransformCounterC.nc"
/*CounterMilliC.Transform*/TransformCounterC$0$upper_count_type /*CounterMilliC.Transform*/TransformCounterC$0$m_upper;

enum /*CounterMilliC.Transform*/TransformCounterC$0$__nesc_unnamed4339 {

  TransformCounterC$0$LOW_SHIFT_RIGHT = 5, 
  TransformCounterC$0$HIGH_SHIFT_LEFT = 8 * sizeof(/*CounterMilliC.Transform*/TransformCounterC$0$from_size_type ) - /*CounterMilliC.Transform*/TransformCounterC$0$LOW_SHIFT_RIGHT, 
  TransformCounterC$0$NUM_UPPER_BITS = 8 * sizeof(/*CounterMilliC.Transform*/TransformCounterC$0$to_size_type ) - 8 * sizeof(/*CounterMilliC.Transform*/TransformCounterC$0$from_size_type ) + 5, 



  TransformCounterC$0$OVERFLOW_MASK = /*CounterMilliC.Transform*/TransformCounterC$0$NUM_UPPER_BITS ? ((/*CounterMilliC.Transform*/TransformCounterC$0$upper_count_type )2 << (/*CounterMilliC.Transform*/TransformCounterC$0$NUM_UPPER_BITS - 1)) - 1 : 0
};

static /*CounterMilliC.Transform*/TransformCounterC$0$to_size_type /*CounterMilliC.Transform*/TransformCounterC$0$Counter$get(void );
#line 112
static inline void /*CounterMilliC.Transform*/TransformCounterC$0$CounterFrom$overflow(void );
# 41 "/Users/jingyuancheng/tinyos/moteiv/tos/lib/timer/CounterToLocalTimeC.nc"
static inline void /*CounterMilliC.CounterToLocalTimeC*/CounterToLocalTimeC$0$Counter$overflow(void );
# 49 "/Users/jingyuancheng/tinyos/moteiv/tos/lib/sched/TaskBasic.nc"
static result_t /*HalTimerMilliC.AlarmToTimerC*/AlarmToTimerC$0$fired$postTask(void );
# 93 "/Users/jingyuancheng/tinyos/moteiv/tos/lib/timer/Alarm.nc"
static /*HalTimerMilliC.AlarmToTimerC*/AlarmToTimerC$0$Alarm$size_type /*HalTimerMilliC.AlarmToTimerC*/AlarmToTimerC$0$Alarm$getNow(void );
#line 88
static void /*HalTimerMilliC.AlarmToTimerC*/AlarmToTimerC$0$Alarm$startAt(/*HalTimerMilliC.AlarmToTimerC*/AlarmToTimerC$0$Alarm$size_type t0, /*HalTimerMilliC.AlarmToTimerC*/AlarmToTimerC$0$Alarm$size_type dt);
#line 60
static void /*HalTimerMilliC.AlarmToTimerC*/AlarmToTimerC$0$Alarm$stop(void );
# 68 "/Users/jingyuancheng/tinyos/moteiv/tos/lib/timer/Timer2.nc"
static void /*HalTimerMilliC.AlarmToTimerC*/AlarmToTimerC$0$Timer$fired(void );
# 57 "/Users/jingyuancheng/tinyos/moteiv/tos/lib/timer/AlarmToTimerC.nc"
enum /*HalTimerMilliC.AlarmToTimerC*/AlarmToTimerC$0$__nesc_unnamed4340 {
#line 57
  AlarmToTimerC$0$fired = 2U
};
#line 57
typedef int /*HalTimerMilliC.AlarmToTimerC*/AlarmToTimerC$0$__nesc_sillytask_fired[/*HalTimerMilliC.AlarmToTimerC*/AlarmToTimerC$0$fired];
#line 34
uint32_t /*HalTimerMilliC.AlarmToTimerC*/AlarmToTimerC$0$m_t0;
uint32_t /*HalTimerMilliC.AlarmToTimerC*/AlarmToTimerC$0$m_dt;
bool /*HalTimerMilliC.AlarmToTimerC*/AlarmToTimerC$0$m_oneshot;

static inline void /*HalTimerMilliC.AlarmToTimerC*/AlarmToTimerC$0$start(uint32_t t0, uint32_t dt, bool oneshot);
#line 53
static inline void /*HalTimerMilliC.AlarmToTimerC*/AlarmToTimerC$0$Timer$stop(void );



static inline void /*HalTimerMilliC.AlarmToTimerC*/AlarmToTimerC$0$fired$runTask(void );





static inline void /*HalTimerMilliC.AlarmToTimerC*/AlarmToTimerC$0$Alarm$fired(void );
#line 79
static inline void /*HalTimerMilliC.AlarmToTimerC*/AlarmToTimerC$0$Timer$startOneShotAt(uint32_t t0, uint32_t dt);



static inline uint32_t /*HalTimerMilliC.AlarmToTimerC*/AlarmToTimerC$0$Timer$getNow(void );



static inline uint32_t /*HalTimerMilliC.AlarmToTimerC*/AlarmToTimerC$0$Timer$gett0(void );



static inline uint32_t /*HalTimerMilliC.AlarmToTimerC*/AlarmToTimerC$0$Timer$getdt(void );
# 116 "/Users/jingyuancheng/tinyos/moteiv/tos/lib/timer/Timer2.nc"
static uint32_t /*HalTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC$0$TimerFrom$getNow(void );
#line 129
static uint32_t /*HalTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC$0$TimerFrom$getdt(void );
#line 123
static uint32_t /*HalTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC$0$TimerFrom$gett0(void );
#line 111
static void /*HalTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC$0$TimerFrom$startOneShotAt(uint32_t t0, uint32_t dt);
#line 64
static void /*HalTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC$0$TimerFrom$stop(void );
# 49 "/Users/jingyuancheng/tinyos/moteiv/tos/lib/sched/TaskBasic.nc"
static result_t /*HalTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC$0$executeTimersNow$postTask(void );
# 68 "/Users/jingyuancheng/tinyos/moteiv/tos/lib/timer/Timer2.nc"
static void /*HalTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC$0$Timer$fired(
# 30 "/Users/jingyuancheng/tinyos/moteiv/tos/lib/timer/VirtualizeTimerC.nc"
uint8_t arg_0x101f29108);
#line 69
enum /*HalTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC$0$__nesc_unnamed4341 {
#line 69
  VirtualizeTimerC$0$executeTimersNow = 3U
};
#line 69
typedef int /*HalTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC$0$__nesc_sillytask_executeTimersNow[/*HalTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC$0$executeTimersNow];
#line 35
typedef result_t /*HalTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC$0$error_t;

enum /*HalTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC$0$__nesc_unnamed4342 {

  VirtualizeTimerC$0$NUM_TIMERS = 5, 
  VirtualizeTimerC$0$END_OF_LIST = 255
};





#line 43
typedef struct /*HalTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC$0$__nesc_unnamed4343 {

  uint32_t t0;
  uint32_t dt;
} /*HalTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC$0$Timer_t;






#line 49
typedef struct /*HalTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC$0$__nesc_unnamed4344 {

  bool isoneshot : 1;
  bool isrunning : 1;
  bool _reserved : 6;
} /*HalTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC$0$Flags_t;

/*HalTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC$0$Timer_t /*HalTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC$0$m_timers[/*HalTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC$0$NUM_TIMERS];
/*HalTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC$0$Flags_t /*HalTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC$0$m_flags[/*HalTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC$0$NUM_TIMERS];

static inline /*HalTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC$0$error_t /*HalTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC$0$Init$init(void );






static inline result_t /*HalTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC$0$Init$start(void );




static void /*HalTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC$0$executeTimers(uint32_t then);
#line 142
static inline void /*HalTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC$0$TimerFrom$fired(void );




static inline void /*HalTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC$0$executeTimersNow$runTask(void );





static void /*HalTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC$0$startTimer(uint8_t num, uint32_t t0, uint32_t dt, bool isoneshot);








static void /*HalTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC$0$Timer$startPeriodic(uint8_t num, uint32_t dt);




static inline void /*HalTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC$0$Timer$startOneShot(uint8_t num, uint32_t dt);




static inline void /*HalTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC$0$Timer$stop(uint8_t num);
# 75 "/Users/jingyuancheng/tinyos/moteiv/tinyos-1.x/tos/interfaces/ReceiveMsg.nc"
static TOS_MsgPtr FramerP$ReceiveMsg$receive(TOS_MsgPtr m);
# 59 "/Users/jingyuancheng/tinyos/moteiv/tinyos-1.x/tos/interfaces/Timer.nc"
static result_t FramerP$Timer$start(char type, uint32_t interval);








static result_t FramerP$Timer$stop(void );
# 23 "/Users/jingyuancheng/tinyos/moteiv/tos/interfaces/Detect.nc"
static bool FramerP$Detect$isConnected(void );
# 49 "/Users/jingyuancheng/tinyos/moteiv/tos/lib/sched/TaskBasic.nc"
static result_t FramerP$PacketUnknown$postTask(void );
#line 49
static result_t FramerP$PacketSent$postTask(void );
# 55 "/Users/jingyuancheng/tinyos/moteiv/tinyos-1.x/tos/interfaces/ByteComm.nc"
static result_t FramerP$ByteComm$txByte(uint8_t data);
# 63 "/Users/jingyuancheng/tinyos/moteiv/tinyos-1.x/tos/interfaces/StdControl.nc"
static result_t FramerP$ByteControl$init(void );






static result_t FramerP$ByteControl$start(void );
# 100 "/Users/jingyuancheng/tinyos/moteiv/tinyos-1.x/tos/platform/msp430/HPLUSARTControl.nc"
static void FramerP$USARTControl$enableUARTRx(void );
#line 90
static void FramerP$USARTControl$enableUARTTx(void );
#line 105
static void FramerP$USARTControl$disableUARTRx(void );
#line 95
static void FramerP$USARTControl$disableUARTTx(void );
# 49 "/Users/jingyuancheng/tinyos/moteiv/tos/lib/sched/TaskBasic.nc"
static result_t FramerP$PacketRcvd$postTask(void );
# 67 "/Users/jingyuancheng/tinyos/moteiv/tinyos-1.x/tos/interfaces/BareSendMsg.nc"
static result_t FramerP$BareSendMsg$sendDone(TOS_MsgPtr msg, result_t success);
# 75 "/Users/jingyuancheng/tinyos/moteiv/tinyos-1.x/tos/interfaces/TokenReceiveMsg.nc"
static TOS_MsgPtr FramerP$TokenReceiveMsg$receive(TOS_MsgPtr Msg, uint8_t Token);
# 155 "/Users/jingyuancheng/tinyos/moteiv/tos/platform/tmote/FramerP.nc"
enum FramerP$__nesc_unnamed4345 {
#line 155
  FramerP$PacketSent = 4U
};
#line 155
typedef int FramerP$__nesc_sillytask_PacketSent[FramerP$PacketSent];
#line 205
enum FramerP$__nesc_unnamed4346 {
#line 205
  FramerP$PacketUnknown = 5U
};
#line 205
typedef int FramerP$__nesc_sillytask_PacketUnknown[FramerP$PacketUnknown];







enum FramerP$__nesc_unnamed4347 {
#line 213
  FramerP$PacketRcvd = 6U
};
#line 213
typedef int FramerP$__nesc_sillytask_PacketRcvd[FramerP$PacketRcvd];
#line 80
enum FramerP$__nesc_unnamed4348 {
  FramerP$HDLC_QUEUESIZE = 2, 
  FramerP$HDLC_MTU = sizeof(TOS_Msg ), 
  FramerP$HDLC_FLAG_BYTE = 0x7e, 
  FramerP$HDLC_CTLESC_BYTE = 0x7d, 
  FramerP$PROTO_ACK = 64, 
  FramerP$PROTO_PACKET_ACK = 65, 
  FramerP$PROTO_PACKET_NOACK = 66, 
  FramerP$PROTO_UNKNOWN = 255
};

enum FramerP$__nesc_unnamed4349 {
  FramerP$RXSTATE_NOSYNC, 
  FramerP$RXSTATE_PROTO, 
  FramerP$RXSTATE_TOKEN, 
  FramerP$RXSTATE_INFO, 
  FramerP$RXSTATE_ESC
};

enum FramerP$__nesc_unnamed4350 {
  FramerP$TXSTATE_IDLE, 
  FramerP$TXSTATE_PROTO, 
  FramerP$TXSTATE_PROTO1, 
  FramerP$TXSTATE_INFO, 
  FramerP$TXSTATE_ESC, 
  FramerP$TXSTATE_FCS1, 
  FramerP$TXSTATE_FCS2, 
  FramerP$TXSTATE_ENDFLAG, 
  FramerP$TXSTATE_FINISH, 
  FramerP$TXSTATE_ERROR
};

enum FramerP$__nesc_unnamed4351 {
  FramerP$FLAGS_TOKENPEND = 0x2, 
  FramerP$FLAGS_DATAPEND = 0x4, 
  FramerP$FLAGS_UNKNOWN = 0x8
};

TOS_Msg FramerP$gMsgRcvBuf[FramerP$HDLC_QUEUESIZE];






#line 120
typedef struct FramerP$_MsgRcvEntry {
  uint8_t Proto;
  uint8_t Token;
  uint16_t Length;
  TOS_MsgPtr pMsg;
} FramerP$MsgRcvEntry_t;

FramerP$MsgRcvEntry_t FramerP$gMsgRcvTbl[FramerP$HDLC_QUEUESIZE];

uint8_t *FramerP$gpRxBuf;
uint8_t *FramerP$gpTxBuf;

uint8_t FramerP$gFlags;


uint8_t FramerP$gTxState;
uint8_t FramerP$gPrevTxState;
uint16_t FramerP$gTxProto;
uint16_t FramerP$gTxByteCnt;
uint16_t FramerP$gTxLength;
uint16_t FramerP$gTxRunningCRC;


uint8_t FramerP$gRxState;
uint8_t FramerP$gRxHeadIndex;
uint8_t FramerP$gRxTailIndex;
uint16_t FramerP$gRxByteCnt;

uint16_t FramerP$gRxRunningCRC;

TOS_MsgPtr FramerP$gpTxMsg;
uint8_t FramerP$gTxTokenBuf;
uint8_t FramerP$gTxUnknownBuf;
uint8_t FramerP$gTxEscByte;



static result_t FramerP$StartTx(void );
#line 205
static inline void FramerP$PacketUnknown$runTask(void );







static inline void FramerP$PacketRcvd$runTask(void );
#line 249
static inline result_t FramerP$Timer$fired(void );




static inline void FramerP$PacketSent$runTask(void );
#line 284
static void FramerP$HDLCInitialize(void );
#line 307
static inline result_t FramerP$StdControl$init(void );




static inline result_t FramerP$StdControl$start(void );
#line 332
static result_t FramerP$BareSendMsg$send(TOS_MsgPtr pMsg);
#line 354
static inline result_t FramerP$TokenReceiveMsg$ReflectToken(uint8_t Token);
#line 374
static inline result_t FramerP$ByteComm$rxByteReady(uint8_t data, bool error, uint16_t strength);
#line 495
static result_t FramerP$TxArbitraryByte(uint8_t inByte);
#line 508
static inline result_t FramerP$ByteComm$txByteReady(bool LastByteSuccess);
#line 584
static inline result_t FramerP$ByteComm$txDone(void );










static inline void FramerP$Detect$connected(void );



static inline void FramerP$Detect$disconnected(void );
# 53 "/Users/jingyuancheng/tinyos/moteiv/tinyos-1.x/tos/platform/msp430/HPLUSARTFeedback.nc"
static result_t HPLUSART1M$USARTData$rxDone(uint8_t data);
#line 46
static result_t HPLUSART1M$USARTData$txDone(void );
# 46 "/Users/jingyuancheng/tinyos/moteiv/tos/platform/msp430/HPLUSART1M.nc"
static volatile uint8_t HPLUSART1M$ME2 __asm ("0x0005");
static volatile uint8_t HPLUSART1M$IFG2 __asm ("0x0003");
static volatile uint8_t HPLUSART1M$U1TCTL __asm ("0x0079");
static volatile uint8_t HPLUSART1M$U1TXBUF __asm ("0x007F");

uint16_t HPLUSART1M$l_br;
uint8_t HPLUSART1M$l_mctl;
uint8_t HPLUSART1M$l_ssel;

void sig_UART1RX_VECTOR(void )  __attribute((wakeup)) __attribute((interrupt(6))) ;




void sig_UART1TX_VECTOR(void )  __attribute((wakeup)) __attribute((interrupt(4))) ;
#line 144
static inline void HPLUSART1M$USARTControl$disableUART(void );





static inline void HPLUSART1M$USARTControl$enableUARTTx(void );




static inline void HPLUSART1M$USARTControl$disableUARTTx(void );




static inline void HPLUSART1M$USARTControl$enableUARTRx(void );




static inline void HPLUSART1M$USARTControl$disableUARTRx(void );
#line 177
static inline void HPLUSART1M$USARTControl$disableSPI(void );







static inline void HPLUSART1M$USARTControl$disableI2C(void );
#line 238
static inline void HPLUSART1M$setUARTModeCommon(void );
#line 314
static inline void HPLUSART1M$USARTControl$setModeUART(void );
#line 332
static inline void HPLUSART1M$USARTControl$setClockSource(uint8_t source);







static inline void HPLUSART1M$USARTControl$setClockRate(uint16_t baudrate, uint8_t mctl);
#line 383
static inline result_t HPLUSART1M$USARTControl$enableRxIntr(void );







static inline result_t HPLUSART1M$USARTControl$enableTxIntr(void );







static inline result_t HPLUSART1M$USARTControl$tx(uint8_t data);
# 49 "/Users/jingyuancheng/tinyos/moteiv/tos/lib/sched/TaskBasic.nc"
static result_t UartPresenceM$taskConnected$postTask(void );
# 31 "/Users/jingyuancheng/tinyos/moteiv/tinyos-1.x/tos/platform/msp430/MSP430GeneralIO.nc"
static bool UartPresenceM$Pin$get(void );
# 33 "/Users/jingyuancheng/tinyos/moteiv/tos/interfaces/Detect.nc"
static void UartPresenceM$Presence$disconnected(void );
#line 28
static void UartPresenceM$Presence$connected(void );
# 49 "/Users/jingyuancheng/tinyos/moteiv/tos/lib/sched/TaskBasic.nc"
static result_t UartPresenceM$taskDisconnected$postTask(void );
# 44 "/Users/jingyuancheng/tinyos/moteiv/tos/platform/msp430/MSP430Interrupt.nc"
static void UartPresenceM$Interrupt$clear(void );
#line 39
static void UartPresenceM$Interrupt$disable(void );
#line 58
static void UartPresenceM$Interrupt$edge(bool low_to_high);
#line 34
static void UartPresenceM$Interrupt$enable(void );
# 40 "/Users/jingyuancheng/tinyos/moteiv/tos/platform/tmote/util/uartdetect/UartPresenceM.nc"
enum UartPresenceM$__nesc_unnamed4352 {
#line 40
  UartPresenceM$taskConnected = 7U
};
#line 40
typedef int UartPresenceM$__nesc_sillytask_taskConnected[UartPresenceM$taskConnected];
enum UartPresenceM$__nesc_unnamed4353 {
#line 41
  UartPresenceM$taskDisconnected = 8U
};
#line 41
typedef int UartPresenceM$__nesc_sillytask_taskDisconnected[UartPresenceM$taskDisconnected];
#line 28
enum UartPresenceM$__nesc_unnamed4354 {
  UartPresenceM$FLAG_CONNECT = 0x01, 
  UartPresenceM$FLAG_EDGE = 0x02, 
  UartPresenceM$FLAG_MSG_BUSY = 0x04
};





bool UartPresenceM$m_flags;




static inline void UartPresenceM$taskConnected$runTask(void );



static inline void UartPresenceM$taskDisconnected$runTask(void );



static inline bool UartPresenceM$Presence$isConnected(void );



static inline result_t UartPresenceM$StdControl$init(void );


static inline result_t UartPresenceM$StdControl$start(void );
#line 79
static inline void UartPresenceM$Interrupt$fired(void );
# 63 "/Users/jingyuancheng/tinyos/moteiv/tos/platform/msp430/MSP430Interrupt.nc"
static void MSP430InterruptM$Port14$fired(void );
#line 63
static void MSP430InterruptM$Port26$fired(void );
#line 63
static void MSP430InterruptM$Port17$fired(void );
#line 63
static void MSP430InterruptM$Port21$fired(void );
#line 63
static void MSP430InterruptM$Port12$fired(void );
#line 63
static void MSP430InterruptM$Port24$fired(void );
#line 63
static void MSP430InterruptM$ACCV$fired(void );
#line 63
static void MSP430InterruptM$Port15$fired(void );
#line 63
static void MSP430InterruptM$Port27$fired(void );
#line 63
static void MSP430InterruptM$Port10$fired(void );
#line 63
static void MSP430InterruptM$Port22$fired(void );
#line 63
static void MSP430InterruptM$OF$fired(void );
#line 63
static void MSP430InterruptM$Port13$fired(void );
#line 63
static void MSP430InterruptM$Port25$fired(void );
#line 63
static void MSP430InterruptM$Port16$fired(void );
#line 63
static void MSP430InterruptM$NMI$fired(void );
#line 63
static void MSP430InterruptM$Port20$fired(void );
#line 63
static void MSP430InterruptM$Port11$fired(void );
#line 63
static void MSP430InterruptM$Port23$fired(void );
# 54 "/Users/jingyuancheng/tinyos/moteiv/tos/platform/msp430/MSP430InterruptM.nc"
static volatile uint8_t MSP430InterruptM$P1IE __asm ("0x0025");
static volatile uint8_t MSP430InterruptM$P2IE __asm ("0x002D");
static volatile uint8_t MSP430InterruptM$P1IFG __asm ("0x0023");
static volatile uint8_t MSP430InterruptM$P2IFG __asm ("0x002B");

void sig_PORT1_VECTOR(void )  __attribute((wakeup)) __attribute((interrupt(8))) ;
#line 74
void sig_PORT2_VECTOR(void )  __attribute((wakeup)) __attribute((interrupt(2))) ;
#line 88
void sig_NMI_VECTOR(void )  __attribute((wakeup)) __attribute((interrupt(28))) ;








static inline void MSP430InterruptM$Port11$default$fired(void );



static inline void MSP430InterruptM$Port15$default$fired(void );
static inline void MSP430InterruptM$Port16$default$fired(void );
static inline void MSP430InterruptM$Port17$default$fired(void );

static inline void MSP430InterruptM$Port20$default$fired(void );
static inline void MSP430InterruptM$Port21$default$fired(void );
static inline void MSP430InterruptM$Port22$default$fired(void );
static inline void MSP430InterruptM$Port23$default$fired(void );
static inline void MSP430InterruptM$Port24$default$fired(void );
static inline void MSP430InterruptM$Port25$default$fired(void );
static inline void MSP430InterruptM$Port26$default$fired(void );
static inline void MSP430InterruptM$Port27$default$fired(void );

static inline void MSP430InterruptM$NMI$default$fired(void );
static inline void MSP430InterruptM$OF$default$fired(void );
static inline void MSP430InterruptM$ACCV$default$fired(void );

static inline void MSP430InterruptM$Port10$enable(void );

static inline void MSP430InterruptM$Port12$enable(void );

static inline void MSP430InterruptM$Port14$enable(void );
#line 149
static inline void MSP430InterruptM$Port10$disable(void );

static inline void MSP430InterruptM$Port12$disable(void );
static inline void MSP430InterruptM$Port13$disable(void );
static inline void MSP430InterruptM$Port14$disable(void );
#line 180
static inline void MSP430InterruptM$Port10$clear(void );
static inline void MSP430InterruptM$Port11$clear(void );
static inline void MSP430InterruptM$Port12$clear(void );
static inline void MSP430InterruptM$Port13$clear(void );
static inline void MSP430InterruptM$Port14$clear(void );
static inline void MSP430InterruptM$Port15$clear(void );
static inline void MSP430InterruptM$Port16$clear(void );
static inline void MSP430InterruptM$Port17$clear(void );

static inline void MSP430InterruptM$Port20$clear(void );
static inline void MSP430InterruptM$Port21$clear(void );
static inline void MSP430InterruptM$Port22$clear(void );
static inline void MSP430InterruptM$Port23$clear(void );
static inline void MSP430InterruptM$Port24$clear(void );
static inline void MSP430InterruptM$Port25$clear(void );
static inline void MSP430InterruptM$Port26$clear(void );
static inline void MSP430InterruptM$Port27$clear(void );

static inline void MSP430InterruptM$NMI$clear(void );
static inline void MSP430InterruptM$OF$clear(void );
static inline void MSP430InterruptM$ACCV$clear(void );
#line 225
static inline void MSP430InterruptM$Port10$edge(bool l2h);
#line 237
static inline void MSP430InterruptM$Port12$edge(bool l2h);
#line 249
static inline void MSP430InterruptM$Port14$edge(bool l2h);
#line 339
static inline void MSP430InterruptM$Port10$makeInput(void );


static inline void MSP430InterruptM$Port13$makeInput(void );
static inline void MSP430InterruptM$Port14$makeInput(void );
# 109 "/Users/jingyuancheng/tinyos/moteiv/tinyos-1.x/tos/platform/msp430/MSP430GeneralIOM.nc"
static inline bool MSP430GeneralIOM$Port12$get(void );
#line 135
static inline void MSP430GeneralIOM$Port15$setHigh(void );
static inline void MSP430GeneralIOM$Port15$setLow(void );




static inline void MSP430GeneralIOM$Port15$makeOutput(void );



static inline void MSP430GeneralIOM$Port16$setHigh(void );
static inline void MSP430GeneralIOM$Port16$setLow(void );




static inline void MSP430GeneralIOM$Port16$makeOutput(void );
#line 165
static inline void MSP430GeneralIOM$Port20$setHigh(void );
static inline void MSP430GeneralIOM$Port20$setLow(void );




static inline void MSP430GeneralIOM$Port20$makeOutput(void );



static inline void MSP430GeneralIOM$Port21$setHigh(void );
static inline void MSP430GeneralIOM$Port21$setLow(void );




static inline void MSP430GeneralIOM$Port21$makeOutput(void );
#line 195
static inline void MSP430GeneralIOM$Port23$setHigh(void );
static inline void MSP430GeneralIOM$Port23$setLow(void );




static inline void MSP430GeneralIOM$Port23$makeOutput(void );
#line 225
static inline void MSP430GeneralIOM$Port26$setHigh(void );
static inline void MSP430GeneralIOM$Port26$setLow(void );




static inline void MSP430GeneralIOM$Port26$makeOutput(void );
#line 445
static inline void MSP430GeneralIOM$Port54$setHigh(void );
static inline void MSP430GeneralIOM$Port54$setLow(void );




static inline void MSP430GeneralIOM$Port54$makeOutput(void );



static inline void MSP430GeneralIOM$Port55$setHigh(void );
static inline void MSP430GeneralIOM$Port55$setLow(void );




static inline void MSP430GeneralIOM$Port55$makeOutput(void );



static inline void MSP430GeneralIOM$Port56$setHigh(void );
static inline void MSP430GeneralIOM$Port56$setLow(void );




static inline void MSP430GeneralIOM$Port56$makeOutput(void );
# 43 "/Users/jingyuancheng/tinyos/moteiv/tos/lib/timer/TimerMilli.nc"
static result_t TimerWrapC$TimerMilli$fired(
# 9 "/Users/jingyuancheng/tinyos/moteiv/tos/lib/timer/TimerWrapC.nc"
uint8_t arg_0x1025b6ab8);
# 52 "/Users/jingyuancheng/tinyos/moteiv/tos/lib/timer/Timer2.nc"
static void TimerWrapC$Timer2$startPeriodic(
# 10 "/Users/jingyuancheng/tinyos/moteiv/tos/lib/timer/TimerWrapC.nc"
uint8_t arg_0x1025b4258, 
# 52 "/Users/jingyuancheng/tinyos/moteiv/tos/lib/timer/Timer2.nc"
uint32_t dt);







static void TimerWrapC$Timer2$startOneShot(
# 10 "/Users/jingyuancheng/tinyos/moteiv/tos/lib/timer/TimerWrapC.nc"
uint8_t arg_0x1025b4258, 
# 60 "/Users/jingyuancheng/tinyos/moteiv/tos/lib/timer/Timer2.nc"
uint32_t dt);



static void TimerWrapC$Timer2$stop(
# 10 "/Users/jingyuancheng/tinyos/moteiv/tos/lib/timer/TimerWrapC.nc"
uint8_t arg_0x1025b4258);
# 73 "/Users/jingyuancheng/tinyos/moteiv/tinyos-1.x/tos/interfaces/Timer.nc"
static result_t TimerWrapC$Timer$fired(
# 8 "/Users/jingyuancheng/tinyos/moteiv/tos/lib/timer/TimerWrapC.nc"
uint8_t arg_0x1025bac10);





enum TimerWrapC$__nesc_unnamed4355 {

  TimerWrapC$TIMER_END = 5U, 
  TimerWrapC$MILLI_BEGIN = TimerWrapC$TIMER_END, 
  TimerWrapC$MILLI_END = TimerWrapC$MILLI_BEGIN + 0U
};




static inline result_t TimerWrapC$Timer$start(uint8_t id, char type, uint32_t interval);
#line 41
static inline result_t TimerWrapC$Timer$stop(uint8_t id);
#line 92
static inline void TimerWrapC$Timer2$fired(uint8_t id);









static inline result_t TimerWrapC$Timer$default$fired(uint8_t id);




static inline result_t TimerWrapC$TimerMilli$default$fired(uint8_t id);
# 49 "/Users/jingyuancheng/tinyos/moteiv/tos/lib/sched/TaskBasic.nc"
static result_t FramerAckM$SendAckTask$postTask(void );
# 75 "/Users/jingyuancheng/tinyos/moteiv/tinyos-1.x/tos/interfaces/ReceiveMsg.nc"
static TOS_MsgPtr FramerAckM$ReceiveCombined$receive(TOS_MsgPtr m);
# 88 "/Users/jingyuancheng/tinyos/moteiv/tinyos-1.x/tos/interfaces/TokenReceiveMsg.nc"
static result_t FramerAckM$TokenReceiveMsg$ReflectToken(uint8_t Token);
# 74 "/Users/jingyuancheng/tinyos/moteiv/tinyos-1.x/tos/system/FramerAckM.nc"
enum FramerAckM$__nesc_unnamed4356 {
#line 74
  FramerAckM$SendAckTask = 9U
};
#line 74
typedef int FramerAckM$__nesc_sillytask_SendAckTask[FramerAckM$SendAckTask];
#line 72
uint8_t FramerAckM$gTokenBuf;

static inline void FramerAckM$SendAckTask$runTask(void );




static inline TOS_MsgPtr FramerAckM$TokenReceiveMsg$receive(TOS_MsgPtr Msg, uint8_t token);
#line 91
static inline TOS_MsgPtr FramerAckM$ReceiveMsg$receive(TOS_MsgPtr Msg);
# 62 "/Users/jingyuancheng/tinyos/moteiv/tinyos-1.x/tos/interfaces/HPLUART.nc"
static result_t UARTM$HPLUART$init(void );
#line 80
static result_t UARTM$HPLUART$put(uint8_t data);
# 83 "/Users/jingyuancheng/tinyos/moteiv/tinyos-1.x/tos/interfaces/ByteComm.nc"
static result_t UARTM$ByteComm$txDone(void );
#line 75
static result_t UARTM$ByteComm$txByteReady(bool success);
#line 66
static result_t UARTM$ByteComm$rxByteReady(uint8_t data, bool error, uint16_t strength);
# 58 "/Users/jingyuancheng/tinyos/moteiv/tinyos-1.x/tos/system/UARTM.nc"
bool UARTM$state;

static inline result_t UARTM$Control$init(void );







static inline result_t UARTM$Control$start(void );








static inline result_t UARTM$HPLUART$get(uint8_t data);









static inline result_t UARTM$HPLUART$putDone(void );
#line 110
static result_t UARTM$ByteComm$txByte(uint8_t data);
# 88 "/Users/jingyuancheng/tinyos/moteiv/tinyos-1.x/tos/interfaces/HPLUART.nc"
static result_t HPLUARTM$UART$get(uint8_t data);







static result_t HPLUARTM$UART$putDone(void );
# 169 "/Users/jingyuancheng/tinyos/moteiv/tinyos-1.x/tos/platform/msp430/HPLUSARTControl.nc"
static void HPLUARTM$USARTControl$setClockRate(uint16_t baudrate, uint8_t mctl);
#line 167
static void HPLUARTM$USARTControl$setClockSource(uint8_t source);






static result_t HPLUARTM$USARTControl$enableRxIntr(void );
static result_t HPLUARTM$USARTControl$enableTxIntr(void );
#line 202
static result_t HPLUARTM$USARTControl$tx(uint8_t data);
#line 153
static void HPLUARTM$USARTControl$setModeUART(void );
# 50 "/Users/jingyuancheng/tinyos/moteiv/tinyos-1.x/tos/platform/msp430/HPLUARTM.nc"
static inline result_t HPLUARTM$UART$init(void );
#line 90
static inline result_t HPLUARTM$USARTData$rxDone(uint8_t b);



static inline result_t HPLUARTM$USARTData$txDone(void );



static inline result_t HPLUARTM$UART$put(uint8_t data);
# 64 "/Users/jingyuancheng/tinyos/moteiv/tinyos-1.x/tos/interfaces/SplitControl.nc"
static result_t CC2420AlwaysOnM$RadioControl$init(void );
#line 77
static result_t CC2420AlwaysOnM$RadioControl$start(void );
# 63 "/Users/jingyuancheng/tinyos/moteiv/tinyos-1.x/tos/interfaces/Random.nc"
static uint16_t CC2420AlwaysOnM$Random$rand(void );
# 31 "/Users/jingyuancheng/tinyos/moteiv/tos/lib/CC2420Radio/MacControl.nc"
static void CC2420AlwaysOnM$MacControl$requestAck(TOS_MsgPtr msg);
#line 22
static void CC2420AlwaysOnM$MacControl$enableAck(void );
# 59 "/Users/jingyuancheng/tinyos/moteiv/tos/lib/CC2420Radio/CC2420BareSendMsg.nc"
static result_t CC2420AlwaysOnM$LowerSend$send(TOS_MsgPtr msg);
# 126 "/Users/jingyuancheng/tinyos/moteiv/tos/lib/sp/SPSend.nc"
static void CC2420AlwaysOnM$SPSend$sendDone(sp_message_t *msg, sp_message_flags_t flags, sp_error_t error);
# 79 "/Users/jingyuancheng/tinyos/moteiv/tos/lib/sp/cc2420/TimeStamping.nc"
static result_t CC2420AlwaysOnM$TimeStamping$addStamp(TOS_MsgPtr msg, int8_t offset);
# 45 "/Users/jingyuancheng/tinyos/moteiv/tos/lib/sp/cc2420/CC2420AlwaysOnM.nc"
enum CC2420AlwaysOnM$__nesc_unnamed4357 {
  CC2420AlwaysOnM$FLAG_SENDDONE = 0x01, 
  CC2420AlwaysOnM$FLAG_MULTIMSG = 0x02
};

sp_message_t *CC2420AlwaysOnM$spmsg;
uint8_t CC2420AlwaysOnM$m_backoffs;
uint8_t CC2420AlwaysOnM$m_flags;

static inline result_t CC2420AlwaysOnM$StdControl$init(void );



static inline result_t CC2420AlwaysOnM$StdControl$start(void );
#line 76
static result_t CC2420AlwaysOnM$SPSend$sendAdv(sp_message_t *_spmsg, TOS_Msg *_tosmsg, sp_device_t _dev, sp_address_t _addr, uint8_t _length, sp_message_flags_t _flags, uint8_t _quantity);
#line 115
static result_t CC2420AlwaysOnM$LowerSend$sendDone(TOS_MsgPtr msg, cc2420_error_t success);
#line 152
static inline sp_linkstate_t CC2420AlwaysOnM$SPLinkStats$getState(void );
#line 167
static inline int16_t CC2420AlwaysOnM$MacBackoff$initialBackoff(TOS_MsgPtr m);





static inline int16_t CC2420AlwaysOnM$MacBackoff$congestionBackoff(TOS_MsgPtr m);




static inline void CC2420AlwaysOnM$SanityTimer$fired(void );

static inline result_t CC2420AlwaysOnM$RadioControl$initDone(void );
static inline result_t CC2420AlwaysOnM$RadioControl$startDone(void );



static inline result_t CC2420AlwaysOnM$RadioControl$stopDone(void );
static inline void CC2420AlwaysOnM$AlarmStart$fired(void );
static inline void CC2420AlwaysOnM$AlarmStop$fired(void );
# 70 "/Users/jingyuancheng/tinyos/moteiv/tinyos-1.x/tos/interfaces/SplitControl.nc"
static result_t CC2420RadioM$SplitControl$initDone(void );
#line 85
static result_t CC2420RadioM$SplitControl$startDone(void );
#line 99
static result_t CC2420RadioM$SplitControl$stopDone(void );
# 49 "/Users/jingyuancheng/tinyos/moteiv/tos/lib/sched/TaskBasic.nc"
static result_t CC2420RadioM$sendFailedTask$postTask(void );
# 59 "/Users/jingyuancheng/tinyos/moteiv/tinyos-1.x/tos/lib/CC2420Radio/HPLCC2420Interrupt.nc"
static result_t CC2420RadioM$FIFOP$disable(void );
#line 43
static result_t CC2420RadioM$FIFOP$startWait(bool low_to_high);
# 65 "/Users/jingyuancheng/tinyos/moteiv/tos/lib/resource/ResourceCmdAsync.nc"
static void CC2420RadioM$CmdFlushRXFIFO$urgentRequest(uint8_t rh);
#line 85
static void CC2420RadioM$CmdFlushRXFIFO$release(void );
# 57 "/Users/jingyuancheng/tinyos/moteiv/tinyos-1.x/tos/interfaces/Random.nc"
static result_t CC2420RadioM$Random$init(void );
# 68 "/Users/jingyuancheng/tinyos/moteiv/tos/lib/CC2420Radio/CC2420BareSendMsg.nc"
static result_t CC2420RadioM$Send$sendDone(TOS_MsgPtr msg, cc2420_error_t success);
# 74 "/Users/jingyuancheng/tinyos/moteiv/tos/lib/timer/Alarm.nc"
static bool CC2420RadioM$BackoffAlarm32khz$isRunning(void );
#line 54
static void CC2420RadioM$BackoffAlarm32khz$start(CC2420RadioM$BackoffAlarm32khz$size_type dt);





static void CC2420RadioM$BackoffAlarm32khz$stop(void );
# 75 "/Users/jingyuancheng/tinyos/moteiv/tos/lib/resource/ResourceCmd.nc"
static void CC2420RadioM$CmdTransmit$release(void );
#line 63
static void CC2420RadioM$CmdTransmit$deferRequest(void );
# 49 "/Users/jingyuancheng/tinyos/moteiv/tos/lib/sched/TaskBasic.nc"
static result_t CC2420RadioM$PacketSent$postTask(void );
# 63 "/Users/jingyuancheng/tinyos/moteiv/tinyos-1.x/tos/interfaces/StdControl.nc"
static result_t CC2420RadioM$TimerControl$init(void );






static result_t CC2420RadioM$TimerControl$start(void );







static result_t CC2420RadioM$TimerControl$stop(void );
# 75 "/Users/jingyuancheng/tinyos/moteiv/tinyos-1.x/tos/interfaces/ReceiveMsg.nc"
static TOS_MsgPtr CC2420RadioM$Receive$receive(TOS_MsgPtr m);
# 49 "/Users/jingyuancheng/tinyos/moteiv/tos/lib/sched/TaskBasic.nc"
static result_t CC2420RadioM$PacketRcvd$postTask(void );
# 71 "/Users/jingyuancheng/tinyos/moteiv/tos/lib/CC2420Radio/HPLCC2420.nc"
static uint16_t CC2420RadioM$HPLChipcon$read(uint8_t rh, uint8_t addr);
#line 48
static uint8_t CC2420RadioM$HPLChipcon$cmd(uint8_t rh, uint8_t addr);
# 33 "/Users/jingyuancheng/tinyos/moteiv/tinyos-1.x/tos/interfaces/RadioCoordinator.nc"
static void CC2420RadioM$RadioReceiveCoordinator$startSymbol(uint8_t bitsPerBlock, uint8_t offset, TOS_MsgPtr msgBuff);
# 60 "/Users/jingyuancheng/tinyos/moteiv/tinyos-1.x/tos/lib/CC2420Radio/HPLCC2420Capture.nc"
static result_t CC2420RadioM$SFD$disable(void );
#line 43
static result_t CC2420RadioM$SFD$enableCapture(bool low_to_high);
# 33 "/Users/jingyuancheng/tinyos/moteiv/tinyos-1.x/tos/interfaces/RadioCoordinator.nc"
static void CC2420RadioM$RadioSendCoordinator$startSymbol(uint8_t bitsPerBlock, uint8_t offset, TOS_MsgPtr msgBuff);
# 70 "/Users/jingyuancheng/tinyos/moteiv/tos/lib/timer/Counter.nc"
static void CC2420RadioM$RadioActiveTime$overflow(void );
# 75 "/Users/jingyuancheng/tinyos/moteiv/tos/lib/resource/ResourceCmd.nc"
static void CC2420RadioM$CmdTryToSend$release(void );
#line 63
static void CC2420RadioM$CmdTryToSend$deferRequest(void );
# 52 "/Users/jingyuancheng/tinyos/moteiv/tos/lib/timer/Counter.nc"
static CC2420RadioM$Counter32khz$size_type CC2420RadioM$Counter32khz$get(void );
# 42 "/Users/jingyuancheng/tinyos/moteiv/tos/lib/CC2420Radio/HPLCC2420FIFO.nc"
static result_t CC2420RadioM$HPLChipconFIFO$writeTXFIFO(uint8_t rh, uint8_t length, uint8_t *data);
#line 30
static result_t CC2420RadioM$HPLChipconFIFO$readRXFIFO(uint8_t rh, uint8_t length, uint8_t *data);
# 75 "/Users/jingyuancheng/tinyos/moteiv/tos/lib/resource/ResourceCmd.nc"
static void CC2420RadioM$CmdReceive$release(void );
#line 63
static void CC2420RadioM$CmdReceive$deferRequest(void );
# 172 "/Users/jingyuancheng/tinyos/moteiv/tos/lib/CC2420Radio/CC2420Control.nc"
static result_t CC2420RadioM$CC2420Control$RxMode(uint8_t rh);
# 19 "/Users/jingyuancheng/tinyos/moteiv/tos/lib/CC2420Radio/MacBackoff.nc"
static int16_t CC2420RadioM$MacBackoff$initialBackoff(TOS_MsgPtr m);
static int16_t CC2420RadioM$MacBackoff$congestionBackoff(TOS_MsgPtr m);
# 49 "/Users/jingyuancheng/tinyos/moteiv/tos/lib/sched/TaskBasic.nc"
static result_t CC2420RadioM$taskShutdownRequest$postTask(void );
# 64 "/Users/jingyuancheng/tinyos/moteiv/tinyos-1.x/tos/interfaces/SplitControl.nc"
static result_t CC2420RadioM$CC2420SplitControl$init(void );
#line 77
static result_t CC2420RadioM$CC2420SplitControl$start(void );
#line 93
static result_t CC2420RadioM$CC2420SplitControl$stop(void );
# 155 "/Users/jingyuancheng/tinyos/moteiv/tos/lib/CC2420Radio/CC2420RadioM.nc"
enum CC2420RadioM$__nesc_unnamed4358 {
#line 155
  CC2420RadioM$sendFailedTask = 10U
};
#line 155
typedef int CC2420RadioM$__nesc_sillytask_sendFailedTask[CC2420RadioM$sendFailedTask];
#line 220
enum CC2420RadioM$__nesc_unnamed4359 {
#line 220
  CC2420RadioM$PacketRcvd = 11U
};
#line 220
typedef int CC2420RadioM$__nesc_sillytask_PacketRcvd[CC2420RadioM$PacketRcvd];
#line 238
enum CC2420RadioM$__nesc_unnamed4360 {
#line 238
  CC2420RadioM$PacketSent = 12U
};
#line 238
typedef int CC2420RadioM$__nesc_sillytask_PacketSent[CC2420RadioM$PacketSent];
#line 250
enum CC2420RadioM$__nesc_unnamed4361 {
#line 250
  CC2420RadioM$taskShutdownRequest = 13U
};
#line 250
typedef int CC2420RadioM$__nesc_sillytask_taskShutdownRequest[CC2420RadioM$taskShutdownRequest];
#line 347
enum CC2420RadioM$__nesc_unnamed4362 {
#line 347
  CC2420RadioM$startRadio = 14U
};
#line 347
typedef int CC2420RadioM$__nesc_sillytask_startRadio[CC2420RadioM$startRadio];
#line 82
enum CC2420RadioM$__nesc_unnamed4363 {
  CC2420RadioM$DISABLED_STATE = 0, 
  CC2420RadioM$DISABLED_STATE_STARTTASK, 
  CC2420RadioM$IDLE_STATE, 
  CC2420RadioM$TX_STATE, 
  CC2420RadioM$TX_WAIT, 
  CC2420RadioM$PRE_TX_STATE, 
  CC2420RadioM$POST_TX_STATE, 
  CC2420RadioM$POST_TX_ACK_STATE, 
  CC2420RadioM$WARMUP_STATE, 

  CC2420RadioM$TIMER_INITIAL = 0, 
  CC2420RadioM$TIMER_BACKOFF, 
  CC2420RadioM$TIMER_ACK, 
  CC2420RadioM$TIMER_SFD
};



enum CC2420RadioM$__nesc_unnamed4364 {
#line 101
  CC2420RadioM$NUM_TIMESTAMPS = 10
};
uint8_t CC2420RadioM$countRetry;
uint8_t CC2420RadioM$stateRadio;
uint8_t CC2420RadioM$stateTimer;
uint8_t CC2420RadioM$currentDSN;
bool CC2420RadioM$bPacketReceiving;
uint8_t CC2420RadioM$txlength;
TOS_MsgPtr CC2420RadioM$txbufptr;
TOS_MsgPtr CC2420RadioM$rxbufptr;
TOS_Msg CC2420RadioM$RxBuf;
uint8_t CC2420RadioM$rh_transmit;
uint8_t CC2420RadioM$rh_receive;
bool CC2420RadioM$m_sfdReceiving;
uint8_t CC2420RadioM$m_rxFifoCount;
bool CC2420RadioM$bShutdownRequest;

uint32_t CC2420RadioM$m_timestamps[CC2420RadioM$NUM_TIMESTAMPS];
CircularQueue_t CC2420RadioM$m_timestampQueue;

volatile uint16_t CC2420RadioM$LocalAddr;

uint32_t CC2420RadioM$cc2420_laston;
uint32_t CC2420RadioM$cc2420_waketime;
#line 138
static inline void CC2420RadioM$RadioActiveTime$default$overflow(void );






static void CC2420RadioM$sendFailedSync(void );









static inline void CC2420RadioM$sendFailedTask$runTask(void );



static inline void CC2420RadioM$sendFailedAsync(void );








static void CC2420RadioM$CmdFlushRXFIFO$granted(uint8_t rh);
#line 188
static inline void CC2420RadioM$flushRXFIFO(uint8_t rh);



static __inline result_t CC2420RadioM$setInitialTimer(uint16_t jiffy);





static __inline result_t CC2420RadioM$setBackoffTimer(uint16_t jiffy);





static __inline result_t CC2420RadioM$setAckTimer(uint16_t jiffy);





static __inline result_t CC2420RadioM$setSFDTimeoutTimer(uint16_t jiffy);









static inline void CC2420RadioM$PacketRcvd$runTask(void );
#line 238
static inline void CC2420RadioM$PacketSent$runTask(void );
#line 250
static inline void CC2420RadioM$taskShutdownRequest$runTask(void );
#line 285
static inline result_t CC2420RadioM$SplitControl$init(void );
#line 304
static inline result_t CC2420RadioM$CC2420SplitControl$initDone(void );
#line 332
static inline result_t CC2420RadioM$CC2420SplitControl$stopDone(void );
#line 347
static inline void CC2420RadioM$startRadio$runTask(void );
#line 385
static result_t CC2420RadioM$SplitControl$start(void );
#line 402
static inline result_t CC2420RadioM$CC2420SplitControl$startDone(void );
#line 432
static inline void CC2420RadioM$sendPacket(uint8_t rh);
#line 458
static inline result_t CC2420RadioM$SFD$captured(uint16_t time);
#line 546
static inline bool CC2420RadioM$startSendBody(uint8_t rh);
#line 565
static inline void CC2420RadioM$CmdTransmit$granted(uint8_t rh);








static void CC2420RadioM$tryToSend(uint8_t rh);
#line 613
static inline void CC2420RadioM$CmdTryToSend$granted(uint8_t rh);








static inline void CC2420RadioM$BackoffAlarm32khz$fired(void );
#line 664
static inline void CC2420RadioM$MacControl$requestAck(TOS_MsgPtr pMsg);
#line 682
static inline result_t CC2420RadioM$Send$send(TOS_MsgPtr pMsg);
#line 722
static inline bool CC2420RadioM$delayedRXFIFOBody(uint8_t rh);
#line 765
static inline void CC2420RadioM$CmdReceive$granted(uint8_t rh);
#line 787
static inline result_t CC2420RadioM$FIFOP$fired(void );
#line 801
static inline result_t CC2420RadioM$doRXFIFODoneBody(uint8_t rh, uint8_t length, uint8_t *data);
#line 904
static inline result_t CC2420RadioM$HPLChipconFIFO$RXFIFODone(uint8_t length, uint8_t *data);





static inline void CC2420RadioM$Counter32khz$overflow(void );


static inline void CC2420RadioM$Counter32khz16$overflow(void );
#line 925
static inline result_t CC2420RadioM$HPLChipconFIFO$TXFIFODone(uint8_t length, uint8_t *data);
#line 940
static inline void CC2420RadioM$MacControl$enableAck(void );
# 70 "/Users/jingyuancheng/tinyos/moteiv/tinyos-1.x/tos/interfaces/SplitControl.nc"
static result_t CC2420ControlM$SplitControl$initDone(void );
#line 85
static result_t CC2420ControlM$SplitControl$startDone(void );
#line 99
static result_t CC2420ControlM$SplitControl$stopDone(void );
# 75 "/Users/jingyuancheng/tinyos/moteiv/tos/lib/resource/ResourceCmd.nc"
static void CC2420ControlM$CmdCCAFired$release(void );
#line 63
static void CC2420ControlM$CmdCCAFired$deferRequest(void );
#line 75
static void CC2420ControlM$CmdSplitControlStop$release(void );
#line 63
static void CC2420ControlM$CmdSplitControlStop$deferRequest(void );
# 71 "/Users/jingyuancheng/tinyos/moteiv/tos/lib/CC2420Radio/HPLCC2420.nc"
static uint16_t CC2420ControlM$HPLChipcon$read(uint8_t rh, uint8_t addr);
#line 60
static uint8_t CC2420ControlM$HPLChipcon$write(uint8_t rh, uint8_t addr, uint16_t data);
#line 48
static uint8_t CC2420ControlM$HPLChipcon$cmd(uint8_t rh, uint8_t addr);
# 59 "/Users/jingyuancheng/tinyos/moteiv/tinyos-1.x/tos/lib/CC2420Radio/HPLCC2420Interrupt.nc"
static result_t CC2420ControlM$CCA$disable(void );
#line 43
static result_t CC2420ControlM$CCA$startWait(bool low_to_high);
# 75 "/Users/jingyuancheng/tinyos/moteiv/tos/lib/resource/ResourceCmd.nc"
static void CC2420ControlM$CmdSplitControlInit$release(void );
#line 63
static void CC2420ControlM$CmdSplitControlInit$deferRequest(void );
# 63 "/Users/jingyuancheng/tinyos/moteiv/tinyos-1.x/tos/interfaces/StdControl.nc"
static result_t CC2420ControlM$HPLChipconControl$init(void );






static result_t CC2420ControlM$HPLChipconControl$start(void );







static result_t CC2420ControlM$HPLChipconControl$stop(void );
# 75 "/Users/jingyuancheng/tinyos/moteiv/tos/lib/resource/ResourceCmd.nc"
static void CC2420ControlM$CmdSplitControlStart$release(void );
#line 63
static void CC2420ControlM$CmdSplitControlStart$deferRequest(void );
# 49 "/Users/jingyuancheng/tinyos/moteiv/tos/lib/CC2420Radio/HPLCC2420RAM.nc"
static result_t CC2420ControlM$HPLChipconRAM$write(uint8_t rh, uint16_t addr, uint8_t length, uint8_t *buffer);
# 85 "/Users/jingyuancheng/tinyos/moteiv/tos/lib/resource/ResourceCmdAsync.nc"
static void CC2420ControlM$CmdCmds$release(void );
#line 58
static void CC2420ControlM$CmdCmds$request(uint8_t rh);
# 55 "/Users/jingyuancheng/tinyos/moteiv/tos/lib/CC2420Radio/CC2420ControlM.nc"
enum CC2420ControlM$__nesc_unnamed4365 {
  CC2420ControlM$IDLE_STATE = 0, 
  CC2420ControlM$INIT_STATE, 
  CC2420ControlM$INIT_STATE_DONE, 
  CC2420ControlM$START_STATE, 
  CC2420ControlM$START_STATE_DONE, 
  CC2420ControlM$STOP_STATE, 

  CC2420ControlM$CMD_OSCILLATOR_ON = 1, 
  CC2420ControlM$CMD_OSCILLATOR_OFF = 2, 

  CC2420ControlM$CMD_SRXON = 1, 
  CC2420ControlM$CMD_STXON = 2, 
  CC2420ControlM$CMD_STXONCCA = 3
};
#line 81
#line 71
typedef union CC2420ControlM$__nesc_unnamed4366 {
  struct  {
    uint8_t freqselect : 1;
    uint8_t setrfpower : 1;
    uint8_t setshortaddress : 1;
    uint8_t mdmctrl0 : 1;
    uint8_t oscillator : 2;
    uint8_t rxtxmode : 2;
  } ;
  uint8_t byte;
} CC2420ControlM$cmds_t;

uint16_t CC2420ControlM$gCurrentParameters[14];
CC2420ControlM$cmds_t CC2420ControlM$cmds = { .byte = 0 };
uint16_t CC2420ControlM$shortAddress = 0;
uint8_t CC2420ControlM$state = 0;

static inline void CC2420ControlM$doCmds(uint8_t rh);






static inline bool CC2420ControlM$SetRegs(uint8_t rh);
#line 129
static inline void CC2420ControlM$CmdSplitControlInit$granted(uint8_t rh);
#line 175
static inline result_t CC2420ControlM$SplitControl$init(void );
#line 192
static inline void CC2420ControlM$CmdSplitControlStop$granted(uint8_t rh);
#line 206
static inline result_t CC2420ControlM$SplitControl$stop(void );
#line 230
static inline void CC2420ControlM$CmdSplitControlStart$granted(uint8_t rh);
#line 245
static inline result_t CC2420ControlM$SplitControl$start(void );
#line 270
static inline void CC2420ControlM$doCmdFreqSelect(uint8_t rh);
#line 294
static inline result_t CC2420ControlM$CC2420Control$TuneManual(uint8_t rh, uint16_t DesiredFreq);
#line 327
static inline void CC2420ControlM$doCmdSTXON(uint8_t rh);
#line 344
static inline void CC2420ControlM$doCmdSTXONCCA(uint8_t rh);
#line 358
static inline void CC2420ControlM$doCmdSRXON(uint8_t rh);



static inline result_t CC2420ControlM$CC2420Control$RxMode(uint8_t rh);
#line 374
static inline void CC2420ControlM$doCmdSetRFPower(uint8_t rh);
#line 393
static inline void CC2420ControlM$doCmdOscillatorOn(uint8_t rh);
#line 414
static inline result_t CC2420ControlM$CC2420Control$OscillatorOn(uint8_t rh);





static inline void CC2420ControlM$doCmdOscillatorOff(uint8_t rh);









static inline result_t CC2420ControlM$CC2420Control$VREFOn(void );






static inline result_t CC2420ControlM$CC2420Control$VREFOff(void );




static inline void CC2420ControlM$doCmdMDMCTRL0(uint8_t rh);
#line 474
static inline void CC2420ControlM$doCmdSetShortAddress(uint8_t rh);



static inline result_t CC2420ControlM$CC2420Control$setShortAddress(uint8_t rh, uint16_t addr);






static inline result_t CC2420ControlM$HPLChipconRAM$readDone(uint16_t addr, uint8_t length, uint8_t *buffer);



static inline result_t CC2420ControlM$HPLChipconRAM$writeDone(uint16_t addr, uint8_t length, uint8_t *buffer);



static inline void CC2420ControlM$CmdCCAFired$granted(uint8_t rh);
#line 506
static inline result_t CC2420ControlM$CCA$fired(void );




static void CC2420ControlM$CmdCmds$granted(uint8_t rh);
#line 533
static inline void CC2420ControlM$doCmds(uint8_t rh);
# 49 "/Users/jingyuancheng/tinyos/moteiv/tos/lib/sched/TaskBasic.nc"
static result_t HPLCC2420M$signalRXFIFO$postTask(void );
#line 49
static result_t HPLCC2420M$signalRAMWr$postTask(void );
#line 49
static result_t HPLCC2420M$signalTXFIFO$postTask(void );
# 28 "/Users/jingyuancheng/tinyos/moteiv/tos/lib/resource/ResourceValidate.nc"
static bool HPLCC2420M$CC2420Validate$validateUser(uint8_t rh);
# 63 "/Users/jingyuancheng/tinyos/moteiv/tos/lib/CC2420Radio/HPLCC2420FIFO.nc"
static result_t HPLCC2420M$HPLCC2420FIFO$TXFIFODone(uint8_t length, uint8_t *data);
#line 52
static result_t HPLCC2420M$HPLCC2420FIFO$RXFIFODone(uint8_t length, uint8_t *data);
# 46 "/Users/jingyuancheng/tinyos/moteiv/tos/lib/sched/Init.nc"
static result_t HPLCC2420M$InterruptInit$init(void );
# 191 "/Users/jingyuancheng/tinyos/moteiv/tinyos-1.x/tos/platform/msp430/HPLUSARTControl.nc"
static result_t HPLCC2420M$USARTControl$isTxEmpty(void );
#line 180
static result_t HPLCC2420M$USARTControl$isTxIntrPending(void );
#line 202
static result_t HPLCC2420M$USARTControl$tx(uint8_t data);






static uint8_t HPLCC2420M$USARTControl$rx(void );
#line 185
static result_t HPLCC2420M$USARTControl$isRxIntrPending(void );
# 51 "/Users/jingyuancheng/tinyos/moteiv/tos/lib/CC2420Radio/HPLCC2420RAM.nc"
static result_t HPLCC2420M$HPLCC2420RAM$writeDone(uint16_t addr, uint8_t length, uint8_t *buffer);
#line 66
static result_t HPLCC2420M$HPLCC2420RAM$readDone(uint16_t addr, uint8_t length, uint8_t *buffer);
# 208 "/Users/jingyuancheng/tinyos/moteiv/tos/platform/tmote/HPLCC2420M.nc"
enum HPLCC2420M$__nesc_unnamed4367 {
#line 208
  HPLCC2420M$signalRAMRd = 15U
};
#line 208
typedef int HPLCC2420M$__nesc_sillytask_signalRAMRd[HPLCC2420M$signalRAMRd];
#line 247
enum HPLCC2420M$__nesc_unnamed4368 {
#line 247
  HPLCC2420M$signalRAMWr = 16U
};
#line 247
typedef int HPLCC2420M$__nesc_sillytask_signalRAMWr[HPLCC2420M$signalRAMWr];
#line 279
enum HPLCC2420M$__nesc_unnamed4369 {
#line 279
  HPLCC2420M$signalRXFIFO = 17U
};
#line 279
typedef int HPLCC2420M$__nesc_sillytask_signalRXFIFO[HPLCC2420M$signalRXFIFO];
#line 332
enum HPLCC2420M$__nesc_unnamed4370 {
#line 332
  HPLCC2420M$signalTXFIFO = 18U
};
#line 332
typedef int HPLCC2420M$__nesc_sillytask_signalTXFIFO[HPLCC2420M$signalTXFIFO];
#line 58
uint8_t *HPLCC2420M$txbuf;
uint8_t *HPLCC2420M$rxbuf;
uint8_t *HPLCC2420M$rambuf;
uint8_t *HPLCC2420M$rxrambuf;
uint8_t HPLCC2420M$txlen;
uint8_t HPLCC2420M$rxlen;
uint8_t HPLCC2420M$ramlen;
uint16_t HPLCC2420M$ramaddr;
uint8_t HPLCC2420M$rxramlen;
uint16_t HPLCC2420M$rxramaddr;

enum HPLCC2420M$__nesc_unnamed4371 {
  HPLCC2420M$IDLE = 0, 
  HPLCC2420M$BUSY_CMD = 1, 
  HPLCC2420M$BUSY_RX = 2, 
  HPLCC2420M$BUSY_TX = 3
};

bool HPLCC2420M$f_enabled;
uint8_t HPLCC2420M$f_busy;





static inline uint8_t HPLCC2420M$adjustStatusByte(uint8_t status);



static inline result_t HPLCC2420M$StdControl$init(void );
#line 99
static inline result_t HPLCC2420M$StdControl$start(void );










static inline result_t HPLCC2420M$StdControl$stop(void );





static bool HPLCC2420M$request(uint8_t rh, uint8_t busy);
#line 128
static inline void HPLCC2420M$release(void );
#line 140
static uint8_t HPLCC2420M$HPLCC2420$cmd(uint8_t rh, uint8_t addr);
#line 161
static uint8_t HPLCC2420M$HPLCC2420$write(uint8_t rh, uint8_t addr, uint16_t data);
#line 186
static uint16_t HPLCC2420M$HPLCC2420$read(uint8_t rh, uint8_t addr);
#line 208
static inline void HPLCC2420M$signalRAMRd$runTask(void );
#line 247
static inline void HPLCC2420M$signalRAMWr$runTask(void );



static result_t HPLCC2420M$HPLCC2420RAM$write(uint8_t rh, uint16_t addr, uint8_t _length, uint8_t *buffer);
#line 279
static inline void HPLCC2420M$signalRXFIFO$runTask(void );
#line 294
static inline result_t HPLCC2420M$HPLCC2420FIFO$readRXFIFO(uint8_t rh, uint8_t length, uint8_t *data);
#line 332
static inline void HPLCC2420M$signalTXFIFO$runTask(void );
#line 355
static inline result_t HPLCC2420M$HPLCC2420FIFO$writeTXFIFO(uint8_t rh, uint8_t length, uint8_t *data);
# 43 "/Users/jingyuancheng/tinyos/moteiv/tinyos-1.x/tos/platform/msp430/HPLI2CInterrupt.nc"
static void HPLUSART0M$HPLI2CInterrupt$fired(void );
# 53 "/Users/jingyuancheng/tinyos/moteiv/tinyos-1.x/tos/platform/msp430/HPLUSARTFeedback.nc"
static result_t HPLUSART0M$USARTData$rxDone(uint8_t data);
#line 46
static result_t HPLUSART0M$USARTData$txDone(void );
# 47 "/Users/jingyuancheng/tinyos/moteiv/tos/platform/msp430/HPLUSART0M.nc"
static volatile uint8_t HPLUSART0M$IE1 __asm ("0x0000");
static volatile uint8_t HPLUSART0M$ME1 __asm ("0x0004");
static volatile uint8_t HPLUSART0M$IFG1 __asm ("0x0002");
static volatile uint8_t HPLUSART0M$U0TCTL __asm ("0x0071");
static volatile uint8_t HPLUSART0M$U0TXBUF __asm ("0x0077");


uint16_t HPLUSART0M$l_br;

uint8_t HPLUSART0M$l_ssel;

void sig_UART0RX_VECTOR(void )  __attribute((wakeup)) __attribute((interrupt(18))) ;




void sig_UART0TX_VECTOR(void )  __attribute((wakeup)) __attribute((interrupt(16))) ;






static inline void HPLUSART0M$HPLI2CInterrupt$default$fired(void );
#line 100
static inline bool HPLUSART0M$USARTControl$isI2C(void );
#line 155
static inline void HPLUSART0M$USARTControl$disableUART(void );
#line 188
static inline void HPLUSART0M$USARTControl$disableSPI(void );
#line 201
static inline void HPLUSART0M$USARTControl$disableI2C(void );






static inline void HPLUSART0M$USARTControl$setModeSPI(void );
#line 411
static result_t HPLUSART0M$USARTControl$isTxIntrPending(void );







static inline result_t HPLUSART0M$USARTControl$isTxEmpty(void );






static result_t HPLUSART0M$USARTControl$isRxIntrPending(void );







static inline result_t HPLUSART0M$USARTControl$disableRxIntr(void );




static inline result_t HPLUSART0M$USARTControl$disableTxIntr(void );
#line 460
static inline result_t HPLUSART0M$USARTControl$tx(uint8_t data);




static uint8_t HPLUSART0M$USARTControl$rx(void );







static inline result_t HPLUSART0M$USARTData$default$txDone(void );

static inline result_t HPLUSART0M$USARTData$default$rxDone(uint8_t data);
# 51 "/Users/jingyuancheng/tinyos/moteiv/tinyos-1.x/tos/lib/CC2420Radio/HPLCC2420Interrupt.nc"
static result_t HPLCC2420InterruptM$FIFO$fired(void );
#line 51
static result_t HPLCC2420InterruptM$FIFOP$fired(void );
# 44 "/Users/jingyuancheng/tinyos/moteiv/tos/platform/msp430/MSP430Interrupt.nc"
static void HPLCC2420InterruptM$CCAInterrupt$clear(void );
#line 69
static void HPLCC2420InterruptM$CCAInterrupt$makeInput(void );
#line 39
static void HPLCC2420InterruptM$CCAInterrupt$disable(void );
#line 58
static void HPLCC2420InterruptM$CCAInterrupt$edge(bool low_to_high);
#line 34
static void HPLCC2420InterruptM$CCAInterrupt$enable(void );
# 36 "/Users/jingyuancheng/tinyos/moteiv/tos/platform/msp430/timer/MSP430TimerControl.nc"
static void HPLCC2420InterruptM$SFDControl$setControlAsCapture(uint8_t capture_mode);

static void HPLCC2420InterruptM$SFDControl$enableEvents(void );
static void HPLCC2420InterruptM$SFDControl$disableEvents(void );
#line 32
static void HPLCC2420InterruptM$SFDControl$clearPendingInterrupt(void );
# 51 "/Users/jingyuancheng/tinyos/moteiv/tinyos-1.x/tos/lib/CC2420Radio/HPLCC2420Interrupt.nc"
static result_t HPLCC2420InterruptM$CCA$fired(void );
# 44 "/Users/jingyuancheng/tinyos/moteiv/tos/platform/msp430/MSP430Interrupt.nc"
static void HPLCC2420InterruptM$FIFOInterrupt$clear(void );
#line 69
static void HPLCC2420InterruptM$FIFOInterrupt$makeInput(void );
#line 39
static void HPLCC2420InterruptM$FIFOInterrupt$disable(void );
# 53 "/Users/jingyuancheng/tinyos/moteiv/tinyos-1.x/tos/lib/CC2420Radio/HPLCC2420Capture.nc"
static result_t HPLCC2420InterruptM$SFD$captured(uint16_t val);
# 56 "/Users/jingyuancheng/tinyos/moteiv/tos/platform/msp430/timer/MSP430Capture.nc"
static void HPLCC2420InterruptM$SFDCapture$clearOverflow(void );
#line 51
static bool HPLCC2420InterruptM$SFDCapture$isOverflowPending(void );
# 44 "/Users/jingyuancheng/tinyos/moteiv/tos/platform/msp430/MSP430Interrupt.nc"
static void HPLCC2420InterruptM$FIFOPInterrupt$clear(void );
#line 69
static void HPLCC2420InterruptM$FIFOPInterrupt$makeInput(void );
#line 39
static void HPLCC2420InterruptM$FIFOPInterrupt$disable(void );
#line 58
static void HPLCC2420InterruptM$FIFOPInterrupt$edge(bool low_to_high);
#line 34
static void HPLCC2420InterruptM$FIFOPInterrupt$enable(void );
# 60 "/Users/jingyuancheng/tinyos/moteiv/tos/platform/tmote/HPLCC2420InterruptM.nc"
static inline result_t HPLCC2420InterruptM$Init$init(void );
#line 73
static result_t HPLCC2420InterruptM$FIFOP$startWait(bool low_to_high);
#line 86
static result_t HPLCC2420InterruptM$FIFOP$disable(void );










static inline void HPLCC2420InterruptM$FIFOPInterrupt$fired(void );
#line 138
static inline void HPLCC2420InterruptM$FIFOInterrupt$fired(void );









static inline result_t HPLCC2420InterruptM$FIFO$default$fired(void );






static inline result_t HPLCC2420InterruptM$CCA$startWait(bool low_to_high);
#line 168
static inline result_t HPLCC2420InterruptM$CCA$disable(void );










static inline void HPLCC2420InterruptM$CCAInterrupt$fired(void );
#line 193
static result_t HPLCC2420InterruptM$SFD$enableCapture(bool low_to_high);
#line 208
static result_t HPLCC2420InterruptM$SFD$disable(void );








static inline void HPLCC2420InterruptM$SFDCapture$captured(uint16_t time);
# 70 "/Users/jingyuancheng/tinyos/moteiv/tos/lib/resource/ResourceCmd.nc"
static void /*MSP430ArbiterUSART0C.ArbiterC.ArbiterP*/FcfsArbiterP$1$ResourceCmd$granted(
# 22 "/Users/jingyuancheng/tinyos/moteiv/tos/lib/resource/FcfsArbiterP.nc"
uint8_t arg_0x101b3b660, 
# 70 "/Users/jingyuancheng/tinyos/moteiv/tos/lib/resource/ResourceCmd.nc"
uint8_t rh);
# 29 "/Users/jingyuancheng/tinyos/moteiv/tos/lib/resource/ResourceConfigure.nc"
static void /*MSP430ArbiterUSART0C.ArbiterC.ArbiterP*/FcfsArbiterP$1$ResourceConfigure$configure(
# 24 "/Users/jingyuancheng/tinyos/moteiv/tos/lib/resource/FcfsArbiterP.nc"
uint8_t arg_0x101b719b0);
# 80 "/Users/jingyuancheng/tinyos/moteiv/tos/lib/resource/ResourceCmdAsync.nc"
static void /*MSP430ArbiterUSART0C.ArbiterC.ArbiterP*/FcfsArbiterP$1$ResourceCmdAsync$granted(
# 23 "/Users/jingyuancheng/tinyos/moteiv/tos/lib/resource/FcfsArbiterP.nc"
uint8_t arg_0x101b3a698, 
# 80 "/Users/jingyuancheng/tinyos/moteiv/tos/lib/resource/ResourceCmdAsync.nc"
uint8_t rh);
# 54 "/Users/jingyuancheng/tinyos/moteiv/tos/lib/resource/Arbiter.nc"
static void /*MSP430ArbiterUSART0C.ArbiterC.ArbiterP*/FcfsArbiterP$1$Arbiter$requested(void );





static void /*MSP430ArbiterUSART0C.ArbiterC.ArbiterP*/FcfsArbiterP$1$Arbiter$idle(void );
# 50 "/Users/jingyuancheng/tinyos/moteiv/tos/lib/sched/TaskBasic.nc"
static result_t /*MSP430ArbiterUSART0C.ArbiterC.ArbiterP*/FcfsArbiterP$1$GrantTask$postUrgentTask(void );
#line 49
static result_t /*MSP430ArbiterUSART0C.ArbiterC.ArbiterP*/FcfsArbiterP$1$GrantTask$postTask(void );
# 73 "/Users/jingyuancheng/tinyos/moteiv/tos/lib/resource/Resource.nc"
static void /*MSP430ArbiterUSART0C.ArbiterC.ArbiterP*/FcfsArbiterP$1$Resource$granted(
# 21 "/Users/jingyuancheng/tinyos/moteiv/tos/lib/resource/FcfsArbiterP.nc"
uint8_t arg_0x101b3c648);







enum /*MSP430ArbiterUSART0C.ArbiterC.ArbiterP*/FcfsArbiterP$1$__nesc_unnamed4372 {
  FcfsArbiterP$1$COUNT = 11U
};





#line 33
struct /*MSP430ArbiterUSART0C.ArbiterC.ArbiterP*/FcfsArbiterP$1$__nesc_unnamed4373 {
  uint8_t head;
  uint8_t tail;
  uint8_t next[/*MSP430ArbiterUSART0C.ArbiterC.ArbiterP*/FcfsArbiterP$1$COUNT];
} /*MSP430ArbiterUSART0C.ArbiterC.ArbiterP*/FcfsArbiterP$1$m_queue;

uint8_t /*MSP430ArbiterUSART0C.ArbiterC.ArbiterP*/FcfsArbiterP$1$m_urgentCount;
uint8_t /*MSP430ArbiterUSART0C.ArbiterC.ArbiterP*/FcfsArbiterP$1$m_granted;


static inline ReservedQueue_t */*MSP430ArbiterUSART0C.ArbiterC.ArbiterP*/FcfsArbiterP$1$queue(void );




static inline result_t /*MSP430ArbiterUSART0C.ArbiterC.ArbiterP*/FcfsArbiterP$1$Init$init(void );








static inline void /*MSP430ArbiterUSART0C.ArbiterC.ArbiterP*/FcfsArbiterP$1$GrantTask$runTask(void );
#line 76
static inline bool /*MSP430ArbiterUSART0C.ArbiterC.ArbiterP*/FcfsArbiterP$1$ResourceValidate$validateUser(uint8_t rh);



static void /*MSP430ArbiterUSART0C.ArbiterC.ArbiterP*/FcfsArbiterP$1$Resource$request(uint8_t id);
#line 105
static void /*MSP430ArbiterUSART0C.ArbiterC.ArbiterP*/FcfsArbiterP$1$Resource$release(uint8_t id);
#line 155
static inline void /*MSP430ArbiterUSART0C.ArbiterC.ArbiterP*/FcfsArbiterP$1$ResourceCmd$deferRequest(uint8_t id);



static inline void /*MSP430ArbiterUSART0C.ArbiterC.ArbiterP*/FcfsArbiterP$1$ResourceCmd$release(uint8_t id);



static void /*MSP430ArbiterUSART0C.ArbiterC.ArbiterP*/FcfsArbiterP$1$ResourceCmdAsync$request(uint8_t id, uint8_t rh);
#line 183
static void /*MSP430ArbiterUSART0C.ArbiterC.ArbiterP*/FcfsArbiterP$1$ResourceCmdAsync$urgentRequest(uint8_t id, uint8_t rh);
#line 219
static inline void /*MSP430ArbiterUSART0C.ArbiterC.ArbiterP*/FcfsArbiterP$1$ResourceCmdAsync$release(uint8_t id);










static inline void /*MSP430ArbiterUSART0C.ArbiterC.ArbiterP*/FcfsArbiterP$1$Resource$default$granted(uint8_t id);


static inline void /*MSP430ArbiterUSART0C.ArbiterC.ArbiterP*/FcfsArbiterP$1$ResourceCmd$default$granted(uint8_t id, uint8_t rh);


static inline void /*MSP430ArbiterUSART0C.ArbiterC.ArbiterP*/FcfsArbiterP$1$ResourceCmdAsync$default$granted(uint8_t id, uint8_t rh);


static void /*MSP430ArbiterUSART0C.ArbiterC.ArbiterP*/FcfsArbiterP$1$ResourceConfigure$default$configure(uint8_t id);
# 115 "/Users/jingyuancheng/tinyos/moteiv/tinyos-1.x/tos/platform/msp430/HPLUSARTControl.nc"
static void /*MSP430ResourceConfigUSART0C.ConfigUSARTP*/MSP430ResourceConfigUSARTP$0$HPLUSARTControl$disableSPI(void );
#line 85
static void /*MSP430ResourceConfigUSART0C.ConfigUSARTP*/MSP430ResourceConfigUSARTP$0$HPLUSARTControl$disableUART(void );
#line 172
static result_t /*MSP430ResourceConfigUSART0C.ConfigUSARTP*/MSP430ResourceConfigUSARTP$0$HPLUSARTControl$disableRxIntr(void );
static result_t /*MSP430ResourceConfigUSART0C.ConfigUSARTP*/MSP430ResourceConfigUSARTP$0$HPLUSARTControl$disableTxIntr(void );
#line 125
static void /*MSP430ResourceConfigUSART0C.ConfigUSARTP*/MSP430ResourceConfigUSARTP$0$HPLUSARTControl$disableI2C(void );









static void /*MSP430ResourceConfigUSART0C.ConfigUSARTP*/MSP430ResourceConfigUSARTP$0$HPLUSARTControl$setModeSPI(void );
# 25 "/Users/jingyuancheng/tinyos/moteiv/tos/platform/msp430/resource/MSP430ResourceConfigUSARTP.nc"
enum /*MSP430ResourceConfigUSART0C.ConfigUSARTP*/MSP430ResourceConfigUSARTP$0$__nesc_unnamed4374 {
  MSP430ResourceConfigUSARTP$0$MODE_UNKNOWN = 0, 
  MSP430ResourceConfigUSARTP$0$MODE_UART, 
  MSP430ResourceConfigUSARTP$0$MODE_SPI, 
  MSP430ResourceConfigUSARTP$0$MODE_I2C
};

uint8_t /*MSP430ResourceConfigUSART0C.ConfigUSARTP*/MSP430ResourceConfigUSARTP$0$m_mode = /*MSP430ResourceConfigUSART0C.ConfigUSARTP*/MSP430ResourceConfigUSARTP$0$MODE_UNKNOWN;
#line 49
static void /*MSP430ResourceConfigUSART0C.ConfigUSARTP*/MSP430ResourceConfigUSARTP$0$ConfigSPI$configure(void );
#line 71
static inline void /*MSP430ResourceConfigUSART0C.ConfigUSARTP*/MSP430ResourceConfigUSARTP$0$Arbiter$idle(void );








static inline void /*MSP430ResourceConfigUSART0C.ConfigUSARTP*/MSP430ResourceConfigUSARTP$0$Arbiter$requested(void );
# 36 "/Users/jingyuancheng/tinyos/moteiv/tos/platform/msp430/RandomMLCG.nc"
uint32_t RandomMLCG$seed;

static inline result_t RandomMLCG$Random$init(void );





static uint16_t RandomMLCG$Random$rand(void );
# 30 "/Users/jingyuancheng/tinyos/moteiv/tos/platform/msp430/timer/MSP430Timer.nc"
static uint16_t /*CC2420RadioC.BackoffAlarm32khzC.MSP430Alarm*/MSP430AlarmC$1$MSP430Timer$get(void );
# 30 "/Users/jingyuancheng/tinyos/moteiv/tos/platform/msp430/timer/MSP430Compare.nc"
static void /*CC2420RadioC.BackoffAlarm32khzC.MSP430Alarm*/MSP430AlarmC$1$MSP430Compare$setEvent(uint16_t time);

static void /*CC2420RadioC.BackoffAlarm32khzC.MSP430Alarm*/MSP430AlarmC$1$MSP430Compare$setEventFromNow(uint16_t delta);
# 64 "/Users/jingyuancheng/tinyos/moteiv/tos/lib/timer/Alarm.nc"
static void /*CC2420RadioC.BackoffAlarm32khzC.MSP430Alarm*/MSP430AlarmC$1$Alarm$fired(void );
# 38 "/Users/jingyuancheng/tinyos/moteiv/tos/platform/msp430/timer/MSP430TimerControl.nc"
static void /*CC2420RadioC.BackoffAlarm32khzC.MSP430Alarm*/MSP430AlarmC$1$MSP430TimerControl$enableEvents(void );

static bool /*CC2420RadioC.BackoffAlarm32khzC.MSP430Alarm*/MSP430AlarmC$1$MSP430TimerControl$areEventsEnabled(void );
#line 35
static void /*CC2420RadioC.BackoffAlarm32khzC.MSP430Alarm*/MSP430AlarmC$1$MSP430TimerControl$setControlAsCompare(void );



static void /*CC2420RadioC.BackoffAlarm32khzC.MSP430Alarm*/MSP430AlarmC$1$MSP430TimerControl$disableEvents(void );
#line 32
static void /*CC2420RadioC.BackoffAlarm32khzC.MSP430Alarm*/MSP430AlarmC$1$MSP430TimerControl$clearPendingInterrupt(void );
# 39 "/Users/jingyuancheng/tinyos/moteiv/tos/platform/msp430/timer/MSP430AlarmC.nc"
typedef result_t /*CC2420RadioC.BackoffAlarm32khzC.MSP430Alarm*/MSP430AlarmC$1$error_t;

static inline /*CC2420RadioC.BackoffAlarm32khzC.MSP430Alarm*/MSP430AlarmC$1$error_t /*CC2420RadioC.BackoffAlarm32khzC.MSP430Alarm*/MSP430AlarmC$1$Init$init(void );






static inline result_t /*CC2420RadioC.BackoffAlarm32khzC.MSP430Alarm*/MSP430AlarmC$1$Init$start(void );
static inline result_t /*CC2420RadioC.BackoffAlarm32khzC.MSP430Alarm*/MSP430AlarmC$1$Init$stop(void );

static inline void /*CC2420RadioC.BackoffAlarm32khzC.MSP430Alarm*/MSP430AlarmC$1$Alarm$start(uint16_t dt);




static inline void /*CC2420RadioC.BackoffAlarm32khzC.MSP430Alarm*/MSP430AlarmC$1$Alarm$stop(void );




static inline void /*CC2420RadioC.BackoffAlarm32khzC.MSP430Alarm*/MSP430AlarmC$1$MSP430Compare$fired(void );





static inline bool /*CC2420RadioC.BackoffAlarm32khzC.MSP430Alarm*/MSP430AlarmC$1$Alarm$isRunning(void );




static void /*CC2420RadioC.BackoffAlarm32khzC.MSP430Alarm*/MSP430AlarmC$1$Alarm$startAt(uint16_t t0, uint16_t dt);
#line 95
static inline uint16_t /*CC2420RadioC.BackoffAlarm32khzC.MSP430Alarm*/MSP430AlarmC$1$Alarm$getNow(void );









static inline void /*CC2420RadioC.BackoffAlarm32khzC.MSP430Alarm*/MSP430AlarmC$1$MSP430Timer$overflow(void );
# 52 "/Users/jingyuancheng/tinyos/moteiv/tos/lib/timer/Counter.nc"
static /*Counter32khzC.Transform*/TransformCounterC$1$CounterFrom$size_type /*Counter32khzC.Transform*/TransformCounterC$1$CounterFrom$get(void );






static bool /*Counter32khzC.Transform*/TransformCounterC$1$CounterFrom$isOverflowPending(void );










static void /*Counter32khzC.Transform*/TransformCounterC$1$Counter$overflow(void );
# 46 "/Users/jingyuancheng/tinyos/moteiv/tos/lib/timer/TransformCounterC.nc"
/*Counter32khzC.Transform*/TransformCounterC$1$upper_count_type /*Counter32khzC.Transform*/TransformCounterC$1$m_upper;

enum /*Counter32khzC.Transform*/TransformCounterC$1$__nesc_unnamed4375 {

  TransformCounterC$1$LOW_SHIFT_RIGHT = 0, 
  TransformCounterC$1$HIGH_SHIFT_LEFT = 8 * sizeof(/*Counter32khzC.Transform*/TransformCounterC$1$from_size_type ) - /*Counter32khzC.Transform*/TransformCounterC$1$LOW_SHIFT_RIGHT, 
  TransformCounterC$1$NUM_UPPER_BITS = 8 * sizeof(/*Counter32khzC.Transform*/TransformCounterC$1$to_size_type ) - 8 * sizeof(/*Counter32khzC.Transform*/TransformCounterC$1$from_size_type ) + 0, 



  TransformCounterC$1$OVERFLOW_MASK = /*Counter32khzC.Transform*/TransformCounterC$1$NUM_UPPER_BITS ? ((/*Counter32khzC.Transform*/TransformCounterC$1$upper_count_type )2 << (/*Counter32khzC.Transform*/TransformCounterC$1$NUM_UPPER_BITS - 1)) - 1 : 0
};

static /*Counter32khzC.Transform*/TransformCounterC$1$to_size_type /*Counter32khzC.Transform*/TransformCounterC$1$Counter$get(void );
#line 112
static inline void /*Counter32khzC.Transform*/TransformCounterC$1$CounterFrom$overflow(void );
# 52 "/Users/jingyuancheng/tinyos/moteiv/tos/lib/timer/Counter.nc"
static /*Counter32khzC.CounterToLocalTimeC*/CounterToLocalTimeC$1$Counter$size_type /*Counter32khzC.CounterToLocalTimeC*/CounterToLocalTimeC$1$Counter$get(void );
# 36 "/Users/jingyuancheng/tinyos/moteiv/tos/lib/timer/CounterToLocalTimeC.nc"
static inline uint32_t /*Counter32khzC.CounterToLocalTimeC*/CounterToLocalTimeC$1$LocalTime$get(void );




static inline void /*Counter32khzC.CounterToLocalTimeC*/CounterToLocalTimeC$1$Counter$overflow(void );
# 49 "/Users/jingyuancheng/tinyos/moteiv/tos/lib/timer/LocalTime.nc"
static uint32_t CC2420TimeStampingM$LocalTime$get(void );
# 65 "/Users/jingyuancheng/tinyos/moteiv/tos/lib/resource/ResourceCmdAsync.nc"
static void CC2420TimeStampingM$CmdWriteTimeStamp$urgentRequest(uint8_t rh);
#line 85
static void CC2420TimeStampingM$CmdWriteTimeStamp$release(void );
# 49 "/Users/jingyuancheng/tinyos/moteiv/tos/lib/CC2420Radio/HPLCC2420RAM.nc"
static result_t CC2420TimeStampingM$HPLCC2420RAM$write(uint8_t rh, uint16_t addr, uint8_t length, uint8_t *buffer);
# 37 "/Users/jingyuancheng/tinyos/moteiv/tos/lib/sp/cc2420/CC2420TimeStampingM.nc"
int8_t CC2420TimeStampingM$sendStampOffset = -1;
TOS_MsgPtr CC2420TimeStampingM$ptosMsg;
TOS_MsgPtr CC2420TimeStampingM$timestampMsgBuf;

enum CC2420TimeStampingM$__nesc_unnamed4376 {
  CC2420TimeStampingM$TX_FIFO_MSG_START = 10, 
  CC2420TimeStampingM$SEND_TIME_CORRECTION = 1
};

static void CC2420TimeStampingM$CmdWriteTimeStamp$granted(uint8_t rh);





static inline void CC2420TimeStampingM$RadioSendCoordinator$startSymbol(uint8_t bitsPerBlock, uint8_t offset, TOS_MsgPtr msgBuff);
#line 71
static inline void CC2420TimeStampingM$RadioReceiveCoordinator$startSymbol(uint8_t bitsPerBlock, uint8_t offset, TOS_MsgPtr msgBuff);
#line 100
static inline result_t CC2420TimeStampingM$TimeStamping$addStamp(TOS_MsgPtr msg, int8_t offset);
#line 114
static inline result_t CC2420TimeStampingM$HPLCC2420RAM$readDone(uint16_t addr, uint8_t length, uint8_t *buffer);



static inline result_t CC2420TimeStampingM$HPLCC2420RAM$writeDone(uint16_t addr, uint8_t length, uint8_t *buffer);
# 64 "/Users/jingyuancheng/tinyos/moteiv/tos/lib/timer/Alarm.nc"
static void /*CC2420SyncAlwaysOnC.DualAlarmC*/VirtualizeAlarmC$0$Alarm$fired(
# 38 "/Users/jingyuancheng/tinyos/moteiv/tos/lib/timer/VirtualizeAlarmC.nc"
uint8_t arg_0x102b44b20);
# 93 "/Users/jingyuancheng/tinyos/moteiv/tos/lib/timer/Alarm.nc"
static /*CC2420SyncAlwaysOnC.DualAlarmC*/VirtualizeAlarmC$0$AlarmFrom$size_type /*CC2420SyncAlwaysOnC.DualAlarmC*/VirtualizeAlarmC$0$AlarmFrom$getNow(void );
#line 88
static void /*CC2420SyncAlwaysOnC.DualAlarmC*/VirtualizeAlarmC$0$AlarmFrom$startAt(/*CC2420SyncAlwaysOnC.DualAlarmC*/VirtualizeAlarmC$0$AlarmFrom$size_type t0, /*CC2420SyncAlwaysOnC.DualAlarmC*/VirtualizeAlarmC$0$AlarmFrom$size_type dt);
#line 60
static void /*CC2420SyncAlwaysOnC.DualAlarmC*/VirtualizeAlarmC$0$AlarmFrom$stop(void );
# 43 "/Users/jingyuancheng/tinyos/moteiv/tos/lib/timer/VirtualizeAlarmC.nc"
typedef result_t /*CC2420SyncAlwaysOnC.DualAlarmC*/VirtualizeAlarmC$0$error_t;

enum /*CC2420SyncAlwaysOnC.DualAlarmC*/VirtualizeAlarmC$0$__nesc_unnamed4377 {
  VirtualizeAlarmC$0$NUM_ALARMS = 2
};




#line 49
typedef struct /*CC2420SyncAlwaysOnC.DualAlarmC*/VirtualizeAlarmC$0$__nesc_unnamed4378 {
  /*CC2420SyncAlwaysOnC.DualAlarmC*/VirtualizeAlarmC$0$size_type t0;
  /*CC2420SyncAlwaysOnC.DualAlarmC*/VirtualizeAlarmC$0$size_type dt;
} /*CC2420SyncAlwaysOnC.DualAlarmC*/VirtualizeAlarmC$0$alarm_t;
#line 71
#line 67
struct /*CC2420SyncAlwaysOnC.DualAlarmC*/VirtualizeAlarmC$0$__nesc_unnamed4379 {
  /*CC2420SyncAlwaysOnC.DualAlarmC*/VirtualizeAlarmC$0$alarm_t alarm[/*CC2420SyncAlwaysOnC.DualAlarmC*/VirtualizeAlarmC$0$NUM_ALARMS];
  bool isset[/*CC2420SyncAlwaysOnC.DualAlarmC*/VirtualizeAlarmC$0$NUM_ALARMS];
  bool is_signaling;
} /*CC2420SyncAlwaysOnC.DualAlarmC*/VirtualizeAlarmC$0$m;

static inline /*CC2420SyncAlwaysOnC.DualAlarmC*/VirtualizeAlarmC$0$error_t /*CC2420SyncAlwaysOnC.DualAlarmC*/VirtualizeAlarmC$0$Init$init(void );




static inline /*CC2420SyncAlwaysOnC.DualAlarmC*/VirtualizeAlarmC$0$error_t /*CC2420SyncAlwaysOnC.DualAlarmC*/VirtualizeAlarmC$0$Init$start(void );







static inline void /*CC2420SyncAlwaysOnC.DualAlarmC*/VirtualizeAlarmC$0$setNextAlarm(void );
#line 138
static inline void /*CC2420SyncAlwaysOnC.DualAlarmC*/VirtualizeAlarmC$0$signalAlarms(void );
#line 166
static inline void /*CC2420SyncAlwaysOnC.DualAlarmC*/VirtualizeAlarmC$0$AlarmFrom$fired(void );
#line 196
static inline void /*CC2420SyncAlwaysOnC.DualAlarmC*/VirtualizeAlarmC$0$Alarm$default$fired(uint8_t id);
# 30 "/Users/jingyuancheng/tinyos/moteiv/tos/platform/msp430/timer/MSP430Timer.nc"
static uint16_t /*CC2420SyncAlwaysOnC.AlarmC.MSP430Alarm*/MSP430AlarmC$2$MSP430Timer$get(void );
# 30 "/Users/jingyuancheng/tinyos/moteiv/tos/platform/msp430/timer/MSP430Compare.nc"
static void /*CC2420SyncAlwaysOnC.AlarmC.MSP430Alarm*/MSP430AlarmC$2$MSP430Compare$setEvent(uint16_t time);

static void /*CC2420SyncAlwaysOnC.AlarmC.MSP430Alarm*/MSP430AlarmC$2$MSP430Compare$setEventFromNow(uint16_t delta);
# 64 "/Users/jingyuancheng/tinyos/moteiv/tos/lib/timer/Alarm.nc"
static void /*CC2420SyncAlwaysOnC.AlarmC.MSP430Alarm*/MSP430AlarmC$2$Alarm$fired(void );
# 38 "/Users/jingyuancheng/tinyos/moteiv/tos/platform/msp430/timer/MSP430TimerControl.nc"
static void /*CC2420SyncAlwaysOnC.AlarmC.MSP430Alarm*/MSP430AlarmC$2$MSP430TimerControl$enableEvents(void );
#line 35
static void /*CC2420SyncAlwaysOnC.AlarmC.MSP430Alarm*/MSP430AlarmC$2$MSP430TimerControl$setControlAsCompare(void );



static void /*CC2420SyncAlwaysOnC.AlarmC.MSP430Alarm*/MSP430AlarmC$2$MSP430TimerControl$disableEvents(void );
#line 32
static void /*CC2420SyncAlwaysOnC.AlarmC.MSP430Alarm*/MSP430AlarmC$2$MSP430TimerControl$clearPendingInterrupt(void );
# 39 "/Users/jingyuancheng/tinyos/moteiv/tos/platform/msp430/timer/MSP430AlarmC.nc"
typedef result_t /*CC2420SyncAlwaysOnC.AlarmC.MSP430Alarm*/MSP430AlarmC$2$error_t;

static inline /*CC2420SyncAlwaysOnC.AlarmC.MSP430Alarm*/MSP430AlarmC$2$error_t /*CC2420SyncAlwaysOnC.AlarmC.MSP430Alarm*/MSP430AlarmC$2$Init$init(void );






static inline result_t /*CC2420SyncAlwaysOnC.AlarmC.MSP430Alarm*/MSP430AlarmC$2$Init$start(void );







static inline void /*CC2420SyncAlwaysOnC.AlarmC.MSP430Alarm*/MSP430AlarmC$2$Alarm$stop(void );




static inline void /*CC2420SyncAlwaysOnC.AlarmC.MSP430Alarm*/MSP430AlarmC$2$MSP430Compare$fired(void );










static void /*CC2420SyncAlwaysOnC.AlarmC.MSP430Alarm*/MSP430AlarmC$2$Alarm$startAt(uint16_t t0, uint16_t dt);
#line 105
static inline void /*CC2420SyncAlwaysOnC.AlarmC.MSP430Alarm*/MSP430AlarmC$2$MSP430Timer$overflow(void );
# 64 "/Users/jingyuancheng/tinyos/moteiv/tos/lib/timer/Alarm.nc"
static void /*CC2420SyncAlwaysOnC.AlarmC.Transform*/TransformAlarmC$1$Alarm$fired(void );
#line 88
static void /*CC2420SyncAlwaysOnC.AlarmC.Transform*/TransformAlarmC$1$AlarmFrom$startAt(/*CC2420SyncAlwaysOnC.AlarmC.Transform*/TransformAlarmC$1$AlarmFrom$size_type t0, /*CC2420SyncAlwaysOnC.AlarmC.Transform*/TransformAlarmC$1$AlarmFrom$size_type dt);
#line 60
static void /*CC2420SyncAlwaysOnC.AlarmC.Transform*/TransformAlarmC$1$AlarmFrom$stop(void );
# 52 "/Users/jingyuancheng/tinyos/moteiv/tos/lib/timer/Counter.nc"
static /*CC2420SyncAlwaysOnC.AlarmC.Transform*/TransformAlarmC$1$Counter$size_type /*CC2420SyncAlwaysOnC.AlarmC.Transform*/TransformAlarmC$1$Counter$get(void );
# 40 "/Users/jingyuancheng/tinyos/moteiv/tos/lib/timer/TransformAlarmC.nc"
/*CC2420SyncAlwaysOnC.AlarmC.Transform*/TransformAlarmC$1$to_size_type /*CC2420SyncAlwaysOnC.AlarmC.Transform*/TransformAlarmC$1$m_t0;
/*CC2420SyncAlwaysOnC.AlarmC.Transform*/TransformAlarmC$1$to_size_type /*CC2420SyncAlwaysOnC.AlarmC.Transform*/TransformAlarmC$1$m_dt;

enum /*CC2420SyncAlwaysOnC.AlarmC.Transform*/TransformAlarmC$1$__nesc_unnamed4380 {

  TransformAlarmC$1$MAX_DELAY_LOG2 = 8 * sizeof(/*CC2420SyncAlwaysOnC.AlarmC.Transform*/TransformAlarmC$1$from_size_type ) - 1 - 0, 
  TransformAlarmC$1$MAX_DELAY = (/*CC2420SyncAlwaysOnC.AlarmC.Transform*/TransformAlarmC$1$to_size_type )1 << /*CC2420SyncAlwaysOnC.AlarmC.Transform*/TransformAlarmC$1$MAX_DELAY_LOG2
};

static inline /*CC2420SyncAlwaysOnC.AlarmC.Transform*/TransformAlarmC$1$to_size_type /*CC2420SyncAlwaysOnC.AlarmC.Transform*/TransformAlarmC$1$Alarm$getNow(void );
#line 65
static inline void /*CC2420SyncAlwaysOnC.AlarmC.Transform*/TransformAlarmC$1$Alarm$stop(void );




static void /*CC2420SyncAlwaysOnC.AlarmC.Transform*/TransformAlarmC$1$set_alarm(void );
#line 100
static inline void /*CC2420SyncAlwaysOnC.AlarmC.Transform*/TransformAlarmC$1$Alarm$startAt(/*CC2420SyncAlwaysOnC.AlarmC.Transform*/TransformAlarmC$1$to_size_type t0, /*CC2420SyncAlwaysOnC.AlarmC.Transform*/TransformAlarmC$1$to_size_type dt);
#line 115
static inline void /*CC2420SyncAlwaysOnC.AlarmC.Transform*/TransformAlarmC$1$AlarmFrom$fired(void );
#line 130
static inline void /*CC2420SyncAlwaysOnC.AlarmC.Transform*/TransformAlarmC$1$Counter$overflow(void );
# 26 "/Users/jingyuancheng/tinyos/moteiv/tos/lib/util/pool/ObjectPoolEvents.nc"
static void /*SPC.MessagePool*/ObjectPoolC$0$PoolEvents$removed(/*SPC.MessagePool*/ObjectPoolC$0$PoolEvents$object_type *object);
#line 20
static void /*SPC.MessagePool*/ObjectPoolC$0$PoolEvents$inserted(/*SPC.MessagePool*/ObjectPoolC$0$PoolEvents$object_type *object);
# 30 "/Users/jingyuancheng/tinyos/moteiv/tos/lib/util/pool/ObjectPoolC.nc"
/*SPC.MessagePool*/ObjectPoolC$0$object_type */*SPC.MessagePool*/ObjectPoolC$0$m_pool[10];

static inline result_t /*SPC.MessagePool*/ObjectPoolC$0$Pool$insert(/*SPC.MessagePool*/ObjectPoolC$0$object_type *obj);
#line 51
static result_t /*SPC.MessagePool*/ObjectPoolC$0$Pool$remove(/*SPC.MessagePool*/ObjectPoolC$0$object_type *obj);
#line 93
static inline /*SPC.MessagePool*/ObjectPoolC$0$object_type */*SPC.MessagePool*/ObjectPoolC$0$Pool$get(uint8_t n);



static inline uint8_t /*SPC.MessagePool*/ObjectPoolC$0$Pool$first(void );



static inline bool /*SPC.MessagePool*/ObjectPoolC$0$Pool$valid(uint8_t n);



static uint8_t /*SPC.MessagePool*/ObjectPoolC$0$Pool$next(uint8_t n);
#line 30
/*SPC.NeighborTable*/ObjectPoolC$1$object_type */*SPC.NeighborTable*/ObjectPoolC$1$m_pool[10];
#line 82
static uint8_t /*SPC.NeighborTable*/ObjectPoolC$1$Pool$populated(void );










static inline /*SPC.NeighborTable*/ObjectPoolC$1$object_type */*SPC.NeighborTable*/ObjectPoolC$1$Pool$get(uint8_t n);



static inline uint8_t /*SPC.NeighborTable*/ObjectPoolC$1$Pool$first(void );



static inline bool /*SPC.NeighborTable*/ObjectPoolC$1$Pool$valid(uint8_t n);



static uint8_t /*SPC.NeighborTable*/ObjectPoolC$1$Pool$next(uint8_t n);
# 75 "/Users/jingyuancheng/tinyos/moteiv/tinyos-1.x/tos/interfaces/ReceiveMsg.nc"
static TOS_MsgPtr SPM$ReceiveMsg$receive(
# 25 "/Users/jingyuancheng/tinyos/moteiv/tos/lib/sp/SPM.nc"
uint8_t arg_0x102c5c618, 
# 75 "/Users/jingyuancheng/tinyos/moteiv/tinyos-1.x/tos/interfaces/ReceiveMsg.nc"
TOS_MsgPtr m);
# 62 "/Users/jingyuancheng/tinyos/moteiv/tos/lib/util/pool/ObjectPool.nc"
static SPM$Pool$object_type *SPM$Pool$get(uint8_t position);
#line 33
static result_t SPM$Pool$remove(SPM$Pool$object_type *obj);
#line 71
static bool SPM$Pool$valid(uint8_t n);
static uint8_t SPM$Pool$next(uint8_t n);
#line 69
static uint8_t SPM$Pool$first(void );
# 49 "/Users/jingyuancheng/tinyos/moteiv/tos/lib/timer/LocalTime.nc"
static uint32_t SPM$LocalTime$get(void );
# 40 "/Users/jingyuancheng/tinyos/moteiv/tos/lib/sp/SPReceive.nc"
static void SPM$SPReceive$receive(
# 24 "/Users/jingyuancheng/tinyos/moteiv/tos/lib/sp/SPM.nc"
uint8_t arg_0x102c5dae8, 
# 40 "/Users/jingyuancheng/tinyos/moteiv/tos/lib/sp/SPReceive.nc"
sp_message_t *spmsg, TOS_MsgPtr tosmsg, sp_error_t result);
# 90 "/Users/jingyuancheng/tinyos/moteiv/tos/lib/sp/SPSend.nc"
static result_t SPM$SPDataMgr$sendAdv(
# 28 "/Users/jingyuancheng/tinyos/moteiv/tos/lib/sp/SPM.nc"
uint8_t arg_0x102c5a258, 
# 90 "/Users/jingyuancheng/tinyos/moteiv/tos/lib/sp/SPSend.nc"
sp_message_t *msg, TOS_Msg *tosmsg, sp_device_t dev, sp_address_t addr, uint8_t length, sp_message_flags_t flags, uint8_t quantity);
# 41 "/Users/jingyuancheng/tinyos/moteiv/tos/lib/sp/SPSendNext.nc"
static void SPM$SPSendNext$request(
# 23 "/Users/jingyuancheng/tinyos/moteiv/tos/lib/sp/SPM.nc"
uint8_t arg_0x102c5ee00, 
# 41 "/Users/jingyuancheng/tinyos/moteiv/tos/lib/sp/SPSendNext.nc"
sp_message_t *msg, TOS_Msg *tosmsg, uint8_t remaining);
# 90 "/Users/jingyuancheng/tinyos/moteiv/tos/lib/sp/SPSend.nc"
static result_t SPM$LowerSend$sendAdv(sp_message_t *msg, TOS_Msg *tosmsg, sp_device_t dev, sp_address_t addr, uint8_t length, sp_message_flags_t flags, uint8_t quantity);
#line 126
static void SPM$SPSend$sendDone(
# 22 "/Users/jingyuancheng/tinyos/moteiv/tos/lib/sp/SPM.nc"
uint8_t arg_0x102c62ca8, 
# 126 "/Users/jingyuancheng/tinyos/moteiv/tos/lib/sp/SPSend.nc"
sp_message_t *msg, sp_message_flags_t flags, sp_error_t error);
# 23 "/Users/jingyuancheng/tinyos/moteiv/tos/lib/sp/SPLinkStats.nc"
static sp_linkstate_t SPM$SPLinkStats$getState(void );
# 22 "/Users/jingyuancheng/tinyos/moteiv/tos/lib/sp/SPNeighbor.nc"
static sp_neighbor_t *SPM$SPNeighbor$get(uint8_t n);









static uint8_t SPM$SPNeighbor$populated(void );
#line 30
static bool SPM$SPNeighbor$valid(uint8_t n);
static uint8_t SPM$SPNeighbor$next(uint8_t n);
#line 29
static uint8_t SPM$SPNeighbor$first(void );
# 43 "/Users/jingyuancheng/tinyos/moteiv/tos/lib/sp/SPM.nc"
sp_message_t *SPM$m_currentmsg;
bool SPM$m_sending;

static void SPM$processSendComplete(sp_message_t *msg, sp_message_flags_t flags, sp_error_t success);

static bool SPM$isOkToSend(uint16_t addr);
#line 62
static inline uint8_t SPM$neighborPopulation(void );



static void SPM$nextSend(void );
#line 146
static inline void SPM$PoolEvents$inserted(sp_message_t *msg);


static inline void SPM$PoolEvents$removed(sp_message_t *msg);


static inline result_t SPM$SPSend$send(uint8_t id, sp_message_t *_msg, TOS_Msg *_tosmsg, sp_address_t _addr, uint8_t _length);









static inline result_t SPM$SPSend$sendAdv(uint8_t id, sp_message_t *_msg, TOS_Msg *_tosmsg, sp_device_t _dev, sp_address_t _addr, uint8_t _length, sp_message_flags_t _flags, uint8_t _quantity);
#line 210
static inline void SPM$SPDataMgrNext$request(uint8_t id, sp_message_t *msg, TOS_Msg *tosmsg, uint8_t quantity);



static inline void SPM$SPDataMgr$sendDone(uint8_t id, sp_message_t *msg, sp_message_flags_t flags, sp_error_t success);



static void SPM$tryNextSend(void );
#line 244
static void SPM$processSendComplete(sp_message_t *msg, sp_message_flags_t flags, sp_error_t success);
#line 311
static inline void SPM$LowerSend$sendDone(sp_message_t *msg, sp_message_flags_t flags, sp_error_t success);
#line 332
static void SPM$setRxFields(sp_message_t *msg, TOS_Msg *tosmsg, sp_device_t dev, sp_message_flags_t flags, uint8_t quantity, uint8_t id);
#line 351
static inline TOS_MsgPtr SPM$LowerReceive$receive(TOS_MsgPtr m);










static TOS_MsgPtr SPM$UARTReceive$receive(TOS_MsgPtr m);
#line 381
static inline void SPM$SPSendNext$default$request(uint8_t id, sp_message_t *msg, TOS_Msg *tosmsg, uint8_t remaining);

static inline void SPM$SPReceive$default$receive(uint8_t id, sp_message_t *spmsg, TOS_MsgPtr m, sp_error_t result);
static inline TOS_MsgPtr SPM$ReceiveMsg$default$receive(uint8_t id, TOS_MsgPtr m);
# 24 "/Users/jingyuancheng/tinyos/moteiv/tos/lib/util/pool/ObjectPool.nc"
static result_t SPDataM$Pool$insert(SPDataM$Pool$object_type *obj);
# 49 "/Users/jingyuancheng/tinyos/moteiv/tos/lib/timer/LocalTime.nc"
static uint32_t SPDataM$LocalTime$get(void );
# 58 "/Users/jingyuancheng/tinyos/moteiv/tinyos-1.x/tos/interfaces/BareSendMsg.nc"
static result_t SPDataM$UARTSend$send(TOS_MsgPtr msg);
# 41 "/Users/jingyuancheng/tinyos/moteiv/tos/lib/sp/SPSendNext.nc"
static void SPDataM$SPSendNext$request(
# 19 "/Users/jingyuancheng/tinyos/moteiv/tos/lib/sp/SPDataM.nc"
uint8_t arg_0x102cba220, 
# 41 "/Users/jingyuancheng/tinyos/moteiv/tos/lib/sp/SPSendNext.nc"
sp_message_t *msg, TOS_Msg *tosmsg, uint8_t remaining);
# 126 "/Users/jingyuancheng/tinyos/moteiv/tos/lib/sp/SPSend.nc"
static void SPDataM$SPSend$sendDone(
# 18 "/Users/jingyuancheng/tinyos/moteiv/tos/lib/sp/SPDataM.nc"
uint8_t arg_0x102cbc0c8, 
# 126 "/Users/jingyuancheng/tinyos/moteiv/tos/lib/sp/SPSend.nc"
sp_message_t *msg, sp_message_flags_t flags, sp_error_t error);
# 29 "/Users/jingyuancheng/tinyos/moteiv/tos/lib/sp/SPDataM.nc"
sp_message_t *SPDataM$m_uartmsg;

static inline void SPDataM$setFields(sp_message_t *msg, TOS_Msg *tosmsg, sp_device_t dev, sp_address_t addr, uint8_t length, sp_message_flags_t flags, uint8_t quantity, uint8_t id);
#line 66
static inline result_t SPDataM$SPSend$sendAdv(uint8_t id, sp_message_t *msg, TOS_Msg *tosmsg, sp_device_t dev, sp_address_t addr, uint8_t length, sp_message_flags_t flags, uint8_t quantity);
#line 113
static inline result_t SPDataM$UARTSend$sendDone(TOS_MsgPtr msg, result_t success);
# 62 "/Users/jingyuancheng/tinyos/moteiv/tos/lib/util/pool/ObjectPool.nc"
static SPNeighborTableM$NeighborTable$object_type *SPNeighborTableM$NeighborTable$get(uint8_t position);
#line 47
static uint8_t SPNeighborTableM$NeighborTable$populated(void );
#line 71
static bool SPNeighborTableM$NeighborTable$valid(uint8_t n);
static uint8_t SPNeighborTableM$NeighborTable$next(uint8_t n);
#line 69
static uint8_t SPNeighborTableM$NeighborTable$first(void );
# 30 "/Users/jingyuancheng/tinyos/moteiv/tos/lib/sp/SPNeighborTableM.nc"
static inline sp_neighbor_t *SPNeighborTableM$SPNeighbor$get(uint8_t id, uint8_t i);





static inline uint8_t SPNeighborTableM$SPNeighbor$first(uint8_t id);


static inline bool SPNeighborTableM$SPNeighbor$valid(uint8_t id, uint8_t i);


static inline uint8_t SPNeighborTableM$SPNeighbor$next(uint8_t id, uint8_t i);


static inline uint8_t SPNeighborTableM$SPNeighbor$populated(uint8_t id);
# 49 "/Users/jingyuancheng/tinyos/moteiv/tinyos-1.x/tos/interfaces/SendMsg.nc"
static result_t SPAdaptorGenericCommM$SendMsg$sendDone(
# 19 "/Users/jingyuancheng/tinyos/moteiv/tos/lib/sp/SPAdaptorGenericCommM.nc"
uint8_t arg_0x102da0790, 
# 49 "/Users/jingyuancheng/tinyos/moteiv/tinyos-1.x/tos/interfaces/SendMsg.nc"
TOS_MsgPtr msg, result_t success);
# 46 "/Users/jingyuancheng/tinyos/moteiv/tos/lib/sp/SPSend.nc"
static result_t SPAdaptorGenericCommM$SPSend$send(
# 22 "/Users/jingyuancheng/tinyos/moteiv/tos/lib/sp/SPAdaptorGenericCommM.nc"
uint8_t arg_0x102d9f488, 
# 46 "/Users/jingyuancheng/tinyos/moteiv/tos/lib/sp/SPSend.nc"
sp_message_t *msg, TOS_Msg *tosmsg, sp_address_t addr, uint8_t length);
# 27 "/Users/jingyuancheng/tinyos/moteiv/tos/lib/sp/SPAdaptorGenericCommM.nc"
sp_message_t SPAdaptorGenericCommM$m_pool[1];

static inline bool SPAdaptorGenericCommM$contains(sp_message_t *_msg);
#line 44
static inline result_t SPAdaptorGenericCommM$SendMsg$send(uint8_t id, uint16_t addr, uint8_t length, TOS_MsgPtr _msg);
#line 65
static void SPAdaptorGenericCommM$SPSend$sendDone(uint8_t id, sp_message_t *_msg, sp_message_flags_t flags, sp_error_t _success);
#line 79
static inline result_t SPAdaptorGenericCommM$SendMsg$default$sendDone(uint8_t id, TOS_MsgPtr p, result_t s);
# 118 "/Users/jingyuancheng/tinyos/moteiv/tos/platform/msp430/adc/RefVolt.nc"
static RefVolt_t MSP430ADC12M$RefVolt$getState(void );
#line 109
static result_t MSP430ADC12M$RefVolt$release(void );
#line 92
static result_t MSP430ADC12M$RefVolt$get(RefVolt_t vref);
# 18 "/Users/jingyuancheng/tinyos/moteiv/tos/platform/msp430/timer/TimerExclusive.nc"
static result_t MSP430ADC12M$TimerExclusive$startTimer(uint8_t rh);



static result_t MSP430ADC12M$TimerExclusive$stopTimer(uint8_t rh);
#line 14
static result_t MSP430ADC12M$TimerExclusive$prepareTimer(uint8_t rh, uint16_t interval, uint16_t csSAMPCON, uint16_t cdSAMPCON);
# 133 "/Users/jingyuancheng/tinyos/moteiv/tos/platform/msp430/adc/MSP430ADC12Single.nc"
static result_t MSP430ADC12M$ADCSingle$dataReady(
# 41 "/Users/jingyuancheng/tinyos/moteiv/tos/platform/msp430/adc/MSP430ADC12M.nc"
uint8_t arg_0x102dc9578, 
# 133 "/Users/jingyuancheng/tinyos/moteiv/tos/platform/msp430/adc/MSP430ADC12Single.nc"
uint16_t data);
# 168 "/Users/jingyuancheng/tinyos/moteiv/tos/platform/msp430/adc/MSP430ADC12Multiple.nc"
static uint16_t *MSP430ADC12M$ADCMultiple$dataReady(
# 42 "/Users/jingyuancheng/tinyos/moteiv/tos/platform/msp430/adc/MSP430ADC12M.nc"
uint8_t arg_0x102dc6220, 
# 168 "/Users/jingyuancheng/tinyos/moteiv/tos/platform/msp430/adc/MSP430ADC12Multiple.nc"
uint16_t *buf, uint16_t length);
# 58 "/Users/jingyuancheng/tinyos/moteiv/tos/platform/msp430/adc/HPLADC12.nc"
static void MSP430ADC12M$HPLADC12$resetIFGs(void );
#line 43
static void MSP430ADC12M$HPLADC12$setControl1(adc12ctl1_t control1);
#line 80
static void MSP430ADC12M$HPLADC12$disableConversion(void );
#line 48
static void MSP430ADC12M$HPLADC12$setControl0_IgnoreRef(adc12ctl0_t control0);


static adc12memctl_t MSP430ADC12M$HPLADC12$getMemControl(uint8_t i);
#line 81
static void MSP430ADC12M$HPLADC12$startConversion(void );
#line 52
static uint16_t MSP430ADC12M$HPLADC12$getMem(uint8_t i);


static void MSP430ADC12M$HPLADC12$setIEFlags(uint16_t mask);
#line 69
static void MSP430ADC12M$HPLADC12$setSHT(uint8_t sht);
#line 50
static void MSP430ADC12M$HPLADC12$setMemControl(uint8_t index, adc12memctl_t memControl);
#line 82
static void MSP430ADC12M$HPLADC12$stopConversion(void );
# 85 "/Users/jingyuancheng/tinyos/moteiv/tos/lib/resource/ResourceCmdAsync.nc"
static void MSP430ADC12M$Resource$release(void );
#line 73
static uint8_t MSP430ADC12M$Resource$immediateRequest(uint8_t rh);
# 55 "/Users/jingyuancheng/tinyos/moteiv/tos/platform/msp430/adc/MSP430ADC12M.nc"
uint8_t MSP430ADC12M$cmode;
uint16_t *MSP430ADC12M$bufPtr;
uint16_t MSP430ADC12M$bufLength;
uint16_t MSP430ADC12M$bufOffset;
uint8_t MSP430ADC12M$owner;
uint8_t MSP430ADC12M$reserved;
uint8_t MSP430ADC12M$vrefWait;
adc12settings_t MSP430ADC12M$adc12settings[8U];

uint8_t MSP430ADC12M$rh;
static void MSP430ADC12M$stopConversion(void );

static inline result_t MSP430ADC12M$StdControl$init(void );









static inline result_t MSP430ADC12M$StdControl$start(void );
#line 90
static inline void MSP430ADC12M$configureAdcPin(uint8_t inputChannel);







static result_t MSP430ADC12M$ADCSingle$bind(uint8_t num, MSP430ADC12Settings_t settings);
#line 131
static inline msp430ADCresult_t MSP430ADC12M$getRefVolt(uint8_t num);
#line 159
static inline result_t MSP430ADC12M$releaseRefVolt(uint8_t num);









static result_t MSP430ADC12M$prepareTimerA(uint16_t interval, uint16_t csSAMPCON, uint16_t cdSAMPCON);









static inline result_t MSP430ADC12M$startTimerA(void );




static msp430ADCresult_t MSP430ADC12M$newRequest(uint8_t req, uint8_t num, void *dataDest, uint16_t length, uint16_t jiffies);
#line 339
static inline msp430ADCresult_t MSP430ADC12M$ADCSingle$getData(uint8_t num);
#line 439
static result_t MSP430ADC12M$ADCSingle$default$dataReady(uint8_t num, uint16_t data);



static inline uint16_t *MSP430ADC12M$ADCMultiple$default$dataReady(uint8_t num, uint16_t *buf, 
uint16_t length);




static void MSP430ADC12M$RefVolt$isStable(RefVolt_t vref);










static void MSP430ADC12M$stopConversion(void );
#line 475
static inline void MSP430ADC12M$HPLADC12$converted(uint8_t number);
#line 529
static inline void MSP430ADC12M$HPLADC12$memOverflow(void );
static inline void MSP430ADC12M$HPLADC12$timeOverflow(void );
static inline void MSP430ADC12M$Resource$granted(uint8_t _rh);
# 61 "/Users/jingyuancheng/tinyos/moteiv/tos/platform/msp430/adc/HPLADC12.nc"
static void HPLADC12M$HPLADC12$memOverflow(void );

static void HPLADC12M$HPLADC12$converted(uint8_t number);
#line 62
static void HPLADC12M$HPLADC12$timeOverflow(void );
# 44 "/Users/jingyuancheng/tinyos/moteiv/tos/platform/msp430/adc/HPLADC12M.nc"
static volatile uint16_t HPLADC12M$ADC12CTL0 __asm ("0x01A0");
static volatile uint16_t HPLADC12M$ADC12CTL1 __asm ("0x01A2");
static volatile uint16_t HPLADC12M$ADC12IFG __asm ("0x01A4");
static volatile uint16_t HPLADC12M$ADC12IE __asm ("0x01A6");
static volatile uint16_t HPLADC12M$ADC12IV __asm ("0x01A8");





static inline void HPLADC12M$HPLADC12$setControl1(adc12ctl1_t control1);



static inline void HPLADC12M$HPLADC12$setControl0_IgnoreRef(adc12ctl0_t control0);
#line 73
static void HPLADC12M$HPLADC12$setMemControl(uint8_t i, adc12memctl_t memControl);







static inline adc12memctl_t HPLADC12M$HPLADC12$getMemControl(uint8_t i);









static inline uint16_t HPLADC12M$HPLADC12$getMem(uint8_t i);



static inline void HPLADC12M$HPLADC12$setIEFlags(uint16_t mask);


static void HPLADC12M$HPLADC12$resetIFGs(void );
#line 112
static inline bool HPLADC12M$HPLADC12$isBusy(void );


static inline void HPLADC12M$HPLADC12$disableConversion(void );
static inline void HPLADC12M$HPLADC12$startConversion(void );
static inline void HPLADC12M$HPLADC12$stopConversion(void );
#line 137
static inline void HPLADC12M$HPLADC12$setRefOn(void );
static inline void HPLADC12M$HPLADC12$setRefOff(void );

static inline void HPLADC12M$HPLADC12$setRef1_5V(void );
static inline void HPLADC12M$HPLADC12$setRef2_5V(void );


static inline void HPLADC12M$HPLADC12$setSHT(uint8_t sht);
#line 163
void sig_ADC_VECTOR(void )  __attribute((wakeup)) __attribute((interrupt(14))) ;
# 28 "/Users/jingyuancheng/tinyos/moteiv/tos/lib/resource/ResourceValidate.nc"
static bool MSP430TimerAExclusiveM$ResourceValidate$validateUser(uint8_t rh);
# 34 "/Users/jingyuancheng/tinyos/moteiv/tos/platform/msp430/timer/MSP430TimerControl.nc"
static void MSP430TimerAExclusiveM$ControlA0$setControl(MSP430CompareControl_t control);
# 30 "/Users/jingyuancheng/tinyos/moteiv/tos/platform/msp430/timer/MSP430Compare.nc"
static void MSP430TimerAExclusiveM$CompareA1$setEvent(uint16_t time);
# 37 "/Users/jingyuancheng/tinyos/moteiv/tos/platform/msp430/timer/MSP430Timer.nc"
static void MSP430TimerAExclusiveM$TimerA$clear(void );


static void MSP430TimerAExclusiveM$TimerA$setClockSource(uint16_t clockSource);
#line 39
static void MSP430TimerAExclusiveM$TimerA$disableEvents(void );
#line 35
static void MSP430TimerAExclusiveM$TimerA$setMode(int mode);





static void MSP430TimerAExclusiveM$TimerA$setInputDivider(uint16_t inputDivider);
# 34 "/Users/jingyuancheng/tinyos/moteiv/tos/platform/msp430/timer/MSP430TimerControl.nc"
static void MSP430TimerAExclusiveM$ControlA1$setControl(MSP430CompareControl_t control);
# 30 "/Users/jingyuancheng/tinyos/moteiv/tos/platform/msp430/timer/MSP430Compare.nc"
static void MSP430TimerAExclusiveM$CompareA0$setEvent(uint16_t time);
# 52 "/Users/jingyuancheng/tinyos/moteiv/tos/platform/msp430/timer/MSP430TimerAExclusiveM.nc"
static inline result_t MSP430TimerAExclusiveM$TimerExclusive$prepareTimer(uint8_t rh, uint16_t interval, uint16_t csSAMPCON, uint16_t cdSAMPCON);
#line 73
static result_t MSP430TimerAExclusiveM$TimerExclusive$startTimer(uint8_t rh);
#line 97
static inline result_t MSP430TimerAExclusiveM$TimerExclusive$stopTimer(uint8_t rh);







static inline void MSP430TimerAExclusiveM$TimerA$overflow(void );
static inline void MSP430TimerAExclusiveM$CompareA0$fired(void );
static inline void MSP430TimerAExclusiveM$CompareA1$fired(void );
# 49 "/Users/jingyuancheng/tinyos/moteiv/tos/lib/sched/TaskBasic.nc"
static result_t RefVoltM$switchOnDelay$postTask(void );
# 60 "/Users/jingyuancheng/tinyos/moteiv/tos/lib/timer/Timer2.nc"
static void RefVoltM$SwitchOffTimer$startOneShot(uint32_t dt);
# 127 "/Users/jingyuancheng/tinyos/moteiv/tos/platform/msp430/adc/RefVolt.nc"
static void RefVoltM$RefVolt$isStable(RefVolt_t vref);
# 49 "/Users/jingyuancheng/tinyos/moteiv/tos/lib/sched/TaskBasic.nc"
static result_t RefVoltM$switchOffDelay$postTask(void );
# 73 "/Users/jingyuancheng/tinyos/moteiv/tos/platform/msp430/adc/HPLADC12.nc"
static void RefVoltM$HPLADC12$setRefOff(void );
#line 65
static bool RefVoltM$HPLADC12$isBusy(void );










static void RefVoltM$HPLADC12$setRef2_5V(void );



static void RefVoltM$HPLADC12$disableConversion(void );
#line 72
static void RefVoltM$HPLADC12$setRefOn(void );


static void RefVoltM$HPLADC12$setRef1_5V(void );
# 49 "/Users/jingyuancheng/tinyos/moteiv/tos/lib/sched/TaskBasic.nc"
static result_t RefVoltM$switchOffRetry$postTask(void );
# 60 "/Users/jingyuancheng/tinyos/moteiv/tos/lib/timer/Timer2.nc"
static void RefVoltM$SwitchOnTimer$startOneShot(uint32_t dt);
# 70 "/Users/jingyuancheng/tinyos/moteiv/tos/platform/msp430/adc/RefVoltM.nc"
enum RefVoltM$__nesc_unnamed4381 {
#line 70
  RefVoltM$switchOnDelay = 20U
};
#line 70
typedef int RefVoltM$__nesc_sillytask_switchOnDelay[RefVoltM$switchOnDelay];
enum RefVoltM$__nesc_unnamed4382 {
#line 71
  RefVoltM$switchOffDelay = 21U
};
#line 71
typedef int RefVoltM$__nesc_sillytask_switchOffDelay[RefVoltM$switchOffDelay];
enum RefVoltM$__nesc_unnamed4383 {
#line 72
  RefVoltM$switchOffRetry = 22U
};
#line 72
typedef int RefVoltM$__nesc_sillytask_switchOffRetry[RefVoltM$switchOffRetry];
#line 52
enum RefVoltM$__nesc_unnamed4384 {

  RefVoltM$REFERENCE_OFF, 
  RefVoltM$REFERENCE_1_5V_PENDING, 
  RefVoltM$REFERENCE_2_5V_PENDING, 
  RefVoltM$REFERENCE_1_5V_STABLE, 
  RefVoltM$REFERENCE_2_5V_STABLE
};

uint8_t RefVoltM$semaCount;
uint8_t RefVoltM$state;
bool RefVoltM$switchOff;

static __inline void RefVoltM$switchRefOn(uint8_t vref);
static __inline void RefVoltM$switchRefOff(void );
static __inline void RefVoltM$switchToRefStable(uint8_t vref);
static __inline void RefVoltM$switchToRefPending(uint8_t vref);





static result_t RefVoltM$RefVolt$get(RefVolt_t vref);
#line 108
static __inline void RefVoltM$switchRefOn(uint8_t vref);
#line 122
static __inline void RefVoltM$switchToRefPending(uint8_t vref);



static __inline void RefVoltM$switchToRefStable(uint8_t vref);



static inline void RefVoltM$switchOnDelay$runTask(void );



static inline void RefVoltM$SwitchOnTimer$fired(void );
#line 147
static inline result_t RefVoltM$RefVolt$release(void );
#line 172
static __inline void RefVoltM$switchRefOff(void );
#line 192
static inline void RefVoltM$switchOffDelay$runTask(void );




static inline void RefVoltM$switchOffRetry$runTask(void );




static inline void RefVoltM$SwitchOffTimer$fired(void );



static inline RefVolt_t RefVoltM$RefVolt$getState(void );







static inline void RefVoltM$HPLADC12$memOverflow(void );
static inline void RefVoltM$HPLADC12$timeOverflow(void );
static inline void RefVoltM$HPLADC12$converted(uint8_t number);
# 207 "/Users/jingyuancheng/tinyos/moteiv/tos/platform/msp430/msp430hardware.h"
static inline bool are_interrupts_enabled()
{
  return (({
#line 209
    uint16_t __x;

#line 209
     __asm volatile ("mov	r2, %0" : "=r"((uint16_t )__x));__x;
  }
  )
#line 209
   & 0x0008) != 0;
}

#line 196
static inline void __nesc_disable_interrupt()
{
   __asm volatile ("dint");
   __asm volatile ("nop");}

# 125 "/Users/jingyuancheng/tinyos/moteiv/tos/platform/msp430/timer/MSP430TimerM.nc"
static inline void /*MSP430TimerC.MSP430TimerA*/MSP430TimerM$0$Overflow$fired(void )
{
  /*MSP430TimerC.MSP430TimerA*/MSP430TimerM$0$Timer$overflow();
}





static inline void /*MSP430TimerC.MSP430TimerA*/MSP430TimerM$0$Event$default$fired(uint8_t n)
{
}

# 4 "/Users/jingyuancheng/tinyos/moteiv/tos/platform/msp430/timer/MSP430TimerEvent.nc"
inline static void /*MSP430TimerC.MSP430TimerA*/MSP430TimerM$0$Event$fired(uint8_t arg_0x101bf0748){
#line 4
  switch (arg_0x101bf0748) {
#line 4
    case 0:
#line 4
      /*MSP430TimerC.MSP430TimerA0*/MSP430TimerCapComM$0$Event$fired();
#line 4
      break;
#line 4
    case 1:
#line 4
      /*MSP430TimerC.MSP430TimerA1*/MSP430TimerCapComM$1$Event$fired();
#line 4
      break;
#line 4
    case 2:
#line 4
      /*MSP430TimerC.MSP430TimerA2*/MSP430TimerCapComM$2$Event$fired();
#line 4
      break;
#line 4
    case 5:
#line 4
      /*MSP430TimerC.MSP430TimerA*/MSP430TimerM$0$Overflow$fired();
#line 4
      break;
#line 4
    default:
#line 4
      /*MSP430TimerC.MSP430TimerA*/MSP430TimerM$0$Event$default$fired(arg_0x101bf0748);
#line 4
      break;
#line 4
    }
#line 4
}
#line 4
# 114 "/Users/jingyuancheng/tinyos/moteiv/tos/platform/msp430/timer/MSP430TimerM.nc"
static inline void /*MSP430TimerC.MSP430TimerA*/MSP430TimerM$0$VectorTimerX0$fired(void )
{
  /*MSP430TimerC.MSP430TimerA*/MSP430TimerM$0$Event$fired(0);
}

# 4 "/Users/jingyuancheng/tinyos/moteiv/tos/platform/msp430/timer/MSP430TimerEvent.nc"
inline static void MSP430TimerCommonM$VectorTimerA0$fired(void ){
#line 4
  /*MSP430TimerC.MSP430TimerA*/MSP430TimerM$0$VectorTimerX0$fired();
#line 4
}
#line 4
# 105 "/Users/jingyuancheng/tinyos/moteiv/tos/platform/msp430/timer/MSP430TimerAExclusiveM.nc"
static inline void MSP430TimerAExclusiveM$TimerA$overflow(void )
#line 105
{
}

# 43 "/Users/jingyuancheng/tinyos/moteiv/tos/platform/msp430/resource/MSP430ResourceConfigTimerAP.nc"
static inline void MSP430ResourceConfigTimerAP$TimerA$overflow(void )
#line 43
{
}

# 32 "/Users/jingyuancheng/tinyos/moteiv/tos/platform/msp430/timer/MSP430DCOCalibP.nc"
static inline void MSP430DCOCalibP$TimerA$overflow(void )
#line 32
{
}

# 184 "/Users/jingyuancheng/tinyos/moteiv/tos/platform/msp430/timer/MSP430TimerCapComM.nc"
static inline void /*MSP430TimerC.MSP430TimerA0*/MSP430TimerCapComM$0$Timer$overflow(void )
{
}

#line 184
static inline void /*MSP430TimerC.MSP430TimerA1*/MSP430TimerCapComM$1$Timer$overflow(void )
{
}

#line 184
static inline void /*MSP430TimerC.MSP430TimerA2*/MSP430TimerCapComM$2$Timer$overflow(void )
{
}

#line 46
static inline /*MSP430TimerC.MSP430TimerA0*/MSP430TimerCapComM$0$CC_t /*MSP430TimerC.MSP430TimerA0*/MSP430TimerCapComM$0$int2CC(uint16_t x)
#line 46
{
#line 46
  union /*MSP430TimerC.MSP430TimerA0*/MSP430TimerCapComM$0$__nesc_unnamed4385 {
#line 46
    uint16_t f;
#line 46
    /*MSP430TimerC.MSP430TimerA0*/MSP430TimerCapComM$0$CC_t t;
  } 
#line 46
  c = { .f = x };

#line 46
  return c.t;
}

#line 73
static inline /*MSP430TimerC.MSP430TimerA0*/MSP430TimerCapComM$0$CC_t /*MSP430TimerC.MSP430TimerA0*/MSP430TimerCapComM$0$Control$getControl(void )
{
  return /*MSP430TimerC.MSP430TimerA0*/MSP430TimerCapComM$0$int2CC(* (volatile uint16_t *)354U);
}

#line 176
static inline void /*MSP430TimerC.MSP430TimerA0*/MSP430TimerCapComM$0$Capture$default$captured(uint16_t n)
{
}

# 74 "/Users/jingyuancheng/tinyos/moteiv/tos/platform/msp430/timer/MSP430Capture.nc"
inline static void /*MSP430TimerC.MSP430TimerA0*/MSP430TimerCapComM$0$Capture$captured(uint16_t time){
#line 74
  /*MSP430TimerC.MSP430TimerA0*/MSP430TimerCapComM$0$Capture$default$captured(time);
#line 74
}
#line 74
# 138 "/Users/jingyuancheng/tinyos/moteiv/tos/platform/msp430/timer/MSP430TimerCapComM.nc"
static inline uint16_t /*MSP430TimerC.MSP430TimerA0*/MSP430TimerCapComM$0$Capture$getEvent(void )
{
  return * (volatile uint16_t *)370U;
}

# 106 "/Users/jingyuancheng/tinyos/moteiv/tos/platform/msp430/timer/MSP430TimerAExclusiveM.nc"
static inline void MSP430TimerAExclusiveM$CompareA0$fired(void )
#line 106
{
}

# 34 "/Users/jingyuancheng/tinyos/moteiv/tos/platform/msp430/timer/MSP430Compare.nc"
inline static void /*MSP430TimerC.MSP430TimerA0*/MSP430TimerCapComM$0$Compare$fired(void ){
#line 34
  MSP430TimerAExclusiveM$CompareA0$fired();
#line 34
}
#line 34
# 46 "/Users/jingyuancheng/tinyos/moteiv/tos/platform/msp430/timer/MSP430TimerCapComM.nc"
static inline /*MSP430TimerC.MSP430TimerA1*/MSP430TimerCapComM$1$CC_t /*MSP430TimerC.MSP430TimerA1*/MSP430TimerCapComM$1$int2CC(uint16_t x)
#line 46
{
#line 46
  union /*MSP430TimerC.MSP430TimerA1*/MSP430TimerCapComM$1$__nesc_unnamed4386 {
#line 46
    uint16_t f;
#line 46
    /*MSP430TimerC.MSP430TimerA1*/MSP430TimerCapComM$1$CC_t t;
  } 
#line 46
  c = { .f = x };

#line 46
  return c.t;
}

#line 73
static inline /*MSP430TimerC.MSP430TimerA1*/MSP430TimerCapComM$1$CC_t /*MSP430TimerC.MSP430TimerA1*/MSP430TimerCapComM$1$Control$getControl(void )
{
  return /*MSP430TimerC.MSP430TimerA1*/MSP430TimerCapComM$1$int2CC(* (volatile uint16_t *)356U);
}

#line 176
static inline void /*MSP430TimerC.MSP430TimerA1*/MSP430TimerCapComM$1$Capture$default$captured(uint16_t n)
{
}

# 74 "/Users/jingyuancheng/tinyos/moteiv/tos/platform/msp430/timer/MSP430Capture.nc"
inline static void /*MSP430TimerC.MSP430TimerA1*/MSP430TimerCapComM$1$Capture$captured(uint16_t time){
#line 74
  /*MSP430TimerC.MSP430TimerA1*/MSP430TimerCapComM$1$Capture$default$captured(time);
#line 74
}
#line 74
# 138 "/Users/jingyuancheng/tinyos/moteiv/tos/platform/msp430/timer/MSP430TimerCapComM.nc"
static inline uint16_t /*MSP430TimerC.MSP430TimerA1*/MSP430TimerCapComM$1$Capture$getEvent(void )
{
  return * (volatile uint16_t *)372U;
}

# 107 "/Users/jingyuancheng/tinyos/moteiv/tos/platform/msp430/timer/MSP430TimerAExclusiveM.nc"
static inline void MSP430TimerAExclusiveM$CompareA1$fired(void )
#line 107
{
}

# 34 "/Users/jingyuancheng/tinyos/moteiv/tos/platform/msp430/timer/MSP430Compare.nc"
inline static void /*MSP430TimerC.MSP430TimerA1*/MSP430TimerCapComM$1$Compare$fired(void ){
#line 34
  MSP430TimerAExclusiveM$CompareA1$fired();
#line 34
}
#line 34
# 46 "/Users/jingyuancheng/tinyos/moteiv/tos/platform/msp430/timer/MSP430TimerCapComM.nc"
static inline /*MSP430TimerC.MSP430TimerA2*/MSP430TimerCapComM$2$CC_t /*MSP430TimerC.MSP430TimerA2*/MSP430TimerCapComM$2$int2CC(uint16_t x)
#line 46
{
#line 46
  union /*MSP430TimerC.MSP430TimerA2*/MSP430TimerCapComM$2$__nesc_unnamed4387 {
#line 46
    uint16_t f;
#line 46
    /*MSP430TimerC.MSP430TimerA2*/MSP430TimerCapComM$2$CC_t t;
  } 
#line 46
  c = { .f = x };

#line 46
  return c.t;
}

#line 73
static inline /*MSP430TimerC.MSP430TimerA2*/MSP430TimerCapComM$2$CC_t /*MSP430TimerC.MSP430TimerA2*/MSP430TimerCapComM$2$Control$getControl(void )
{
  return /*MSP430TimerC.MSP430TimerA2*/MSP430TimerCapComM$2$int2CC(* (volatile uint16_t *)358U);
}

#line 176
static inline void /*MSP430TimerC.MSP430TimerA2*/MSP430TimerCapComM$2$Capture$default$captured(uint16_t n)
{
}

# 74 "/Users/jingyuancheng/tinyos/moteiv/tos/platform/msp430/timer/MSP430Capture.nc"
inline static void /*MSP430TimerC.MSP430TimerA2*/MSP430TimerCapComM$2$Capture$captured(uint16_t time){
#line 74
  /*MSP430TimerC.MSP430TimerA2*/MSP430TimerCapComM$2$Capture$default$captured(time);
#line 74
}
#line 74
# 138 "/Users/jingyuancheng/tinyos/moteiv/tos/platform/msp430/timer/MSP430TimerCapComM.nc"
static inline uint16_t /*MSP430TimerC.MSP430TimerA2*/MSP430TimerCapComM$2$Capture$getEvent(void )
{
  return * (volatile uint16_t *)374U;
}

#line 180
static inline void /*MSP430TimerC.MSP430TimerA2*/MSP430TimerCapComM$2$Compare$default$fired(void )
{
}

# 34 "/Users/jingyuancheng/tinyos/moteiv/tos/platform/msp430/timer/MSP430Compare.nc"
inline static void /*MSP430TimerC.MSP430TimerA2*/MSP430TimerCapComM$2$Compare$fired(void ){
#line 34
  /*MSP430TimerC.MSP430TimerA2*/MSP430TimerCapComM$2$Compare$default$fired();
#line 34
}
#line 34
# 119 "/Users/jingyuancheng/tinyos/moteiv/tos/platform/msp430/timer/MSP430TimerM.nc"
static inline void /*MSP430TimerC.MSP430TimerA*/MSP430TimerM$0$VectorTimerX1$fired(void )
{
  uint8_t n = * (volatile uint16_t *)302U;

#line 122
  /*MSP430TimerC.MSP430TimerA*/MSP430TimerM$0$Event$fired(n >> 1);
}

# 4 "/Users/jingyuancheng/tinyos/moteiv/tos/platform/msp430/timer/MSP430TimerEvent.nc"
inline static void MSP430TimerCommonM$VectorTimerA1$fired(void ){
#line 4
  /*MSP430TimerC.MSP430TimerA*/MSP430TimerM$0$VectorTimerX1$fired();
#line 4
}
#line 4
# 114 "/Users/jingyuancheng/tinyos/moteiv/tos/platform/msp430/timer/MSP430TimerM.nc"
static inline void /*MSP430TimerC.MSP430TimerB*/MSP430TimerM$1$VectorTimerX0$fired(void )
{
  /*MSP430TimerC.MSP430TimerB*/MSP430TimerM$1$Event$fired(0);
}

# 4 "/Users/jingyuancheng/tinyos/moteiv/tos/platform/msp430/timer/MSP430TimerEvent.nc"
inline static void MSP430TimerCommonM$VectorTimerB0$fired(void ){
#line 4
  /*MSP430TimerC.MSP430TimerB*/MSP430TimerM$1$VectorTimerX0$fired();
#line 4
}
#line 4
# 184 "/Users/jingyuancheng/tinyos/moteiv/tos/platform/msp430/timer/MSP430TimerCapComM.nc"
static inline void /*MSP430TimerC.MSP430TimerB6*/MSP430TimerCapComM$9$Timer$overflow(void )
{
}

#line 184
static inline void /*MSP430TimerC.MSP430TimerB5*/MSP430TimerCapComM$8$Timer$overflow(void )
{
}

#line 184
static inline void /*MSP430TimerC.MSP430TimerB4*/MSP430TimerCapComM$7$Timer$overflow(void )
{
}

#line 184
static inline void /*MSP430TimerC.MSP430TimerB3*/MSP430TimerCapComM$6$Timer$overflow(void )
{
}

#line 184
static inline void /*MSP430TimerC.MSP430TimerB2*/MSP430TimerCapComM$5$Timer$overflow(void )
{
}

#line 184
static inline void /*MSP430TimerC.MSP430TimerB1*/MSP430TimerCapComM$4$Timer$overflow(void )
{
}

#line 184
static inline void /*MSP430TimerC.MSP430TimerB0*/MSP430TimerCapComM$3$Timer$overflow(void )
{
}

# 105 "/Users/jingyuancheng/tinyos/moteiv/tos/platform/msp430/timer/MSP430AlarmC.nc"
static inline void /*CC2420SyncAlwaysOnC.AlarmC.MSP430Alarm*/MSP430AlarmC$2$MSP430Timer$overflow(void )
{
}

#line 105
static inline void /*CC2420RadioC.BackoffAlarm32khzC.MSP430Alarm*/MSP430AlarmC$1$MSP430Timer$overflow(void )
{
}

#line 105
static inline void /*HalTimerMilliC.AlarmMilliC.MSP430Alarm*/MSP430AlarmC$0$MSP430Timer$overflow(void )
{
}

# 43 "/Users/jingyuancheng/tinyos/moteiv/tos/lib/resource/FcfsArbiterP.nc"
static inline ReservedQueue_t */*MSP430ArbiterTimerAC.ArbiterC.ArbiterP*/FcfsArbiterP$0$queue(void )
#line 43
{
  return (ReservedQueue_t *)&/*MSP430ArbiterTimerAC.ArbiterC.ArbiterP*/FcfsArbiterP$0$m_queue;
}

# 124 "/Users/jingyuancheng/tinyos/moteiv/tos/lib/util/reservedQueue.h"
static inline bool rqueue_push(ReservedQueue_t *q, uint8_t id)
#line 124
{
  return rqueue_push_priv(q, id, FALSE);
}

# 80 "/Users/jingyuancheng/tinyos/moteiv/tos/lib/resource/FcfsArbiterP.nc"
static inline void /*MSP430ArbiterTimerAC.ArbiterC.ArbiterP*/FcfsArbiterP$0$Resource$request(uint8_t id)
#line 80
{
  /* atomic removed: atomic calls only */
#line 81
  {
    if (/*MSP430ArbiterTimerAC.ArbiterC.ArbiterP*/FcfsArbiterP$0$m_granted == RESOURCE_NONE) {
        /*MSP430ArbiterTimerAC.ArbiterC.ArbiterP*/FcfsArbiterP$0$m_granted = id;
      }
    else {
        rqueue_push(/*MSP430ArbiterTimerAC.ArbiterC.ArbiterP*/FcfsArbiterP$0$queue(), id);
        return;
      }
  }
  /*MSP430ArbiterTimerAC.ArbiterC.ArbiterP*/FcfsArbiterP$0$GrantTask$postTask();
}

#line 155
static inline void /*MSP430ArbiterTimerAC.ArbiterC.ArbiterP*/FcfsArbiterP$0$ResourceCmd$deferRequest(uint8_t id)
#line 155
{
  /*MSP430ArbiterTimerAC.ArbiterC.ArbiterP*/FcfsArbiterP$0$Resource$request(id);
}

# 63 "/Users/jingyuancheng/tinyos/moteiv/tos/lib/resource/ResourceCmd.nc"
inline static void MSP430DCOCalibP$ResourceTimerA$deferRequest(void ){
#line 63
  /*MSP430ArbiterTimerAC.ArbiterC.ArbiterP*/FcfsArbiterP$0$ResourceCmd$deferRequest(/*MSP430DCOCalibC.MSP430ResourceTimerAC*/MSP430ResourceTimerAC$0$ID);
#line 63
}
#line 63
# 36 "/Users/jingyuancheng/tinyos/moteiv/tos/platform/msp430/timer/MSP430DCOCalibP.nc"
static inline void MSP430DCOCalibP$TimerB$overflow(void )
#line 36
{
  MSP430DCOCalibP$ResourceTimerA$deferRequest();
}

# 41 "/Users/jingyuancheng/tinyos/moteiv/tos/lib/timer/CounterToLocalTimeC.nc"
static inline void /*CounterMilliC.CounterToLocalTimeC*/CounterToLocalTimeC$0$Counter$overflow(void )
{
}

# 130 "/Users/jingyuancheng/tinyos/moteiv/tos/lib/timer/TransformAlarmC.nc"
static inline void /*HalTimerMilliC.AlarmMilliC.Transform*/TransformAlarmC$0$Counter$overflow(void )
{
}

# 70 "/Users/jingyuancheng/tinyos/moteiv/tos/lib/timer/Counter.nc"
inline static void /*CounterMilliC.Transform*/TransformCounterC$0$Counter$overflow(void ){
#line 70
  /*HalTimerMilliC.AlarmMilliC.Transform*/TransformAlarmC$0$Counter$overflow();
#line 70
  /*CounterMilliC.CounterToLocalTimeC*/CounterToLocalTimeC$0$Counter$overflow();
#line 70
}
#line 70
# 112 "/Users/jingyuancheng/tinyos/moteiv/tos/lib/timer/TransformCounterC.nc"
static inline void /*CounterMilliC.Transform*/TransformCounterC$0$CounterFrom$overflow(void )
{
  /* atomic removed: atomic calls only */
  {
    /*CounterMilliC.Transform*/TransformCounterC$0$m_upper++;
    if ((/*CounterMilliC.Transform*/TransformCounterC$0$m_upper & /*CounterMilliC.Transform*/TransformCounterC$0$OVERFLOW_MASK) == 0) {
      /*CounterMilliC.Transform*/TransformCounterC$0$Counter$overflow();
      }
  }
}

# 41 "/Users/jingyuancheng/tinyos/moteiv/tos/lib/timer/CounterToLocalTimeC.nc"
static inline void /*Counter32khzC.CounterToLocalTimeC*/CounterToLocalTimeC$1$Counter$overflow(void )
{
}

# 910 "/Users/jingyuancheng/tinyos/moteiv/tos/lib/CC2420Radio/CC2420RadioM.nc"
static inline void CC2420RadioM$Counter32khz$overflow(void )
#line 910
{
}

# 130 "/Users/jingyuancheng/tinyos/moteiv/tos/lib/timer/TransformAlarmC.nc"
static inline void /*CC2420SyncAlwaysOnC.AlarmC.Transform*/TransformAlarmC$1$Counter$overflow(void )
{
}

# 70 "/Users/jingyuancheng/tinyos/moteiv/tos/lib/timer/Counter.nc"
inline static void /*Counter32khzC.Transform*/TransformCounterC$1$Counter$overflow(void ){
#line 70
  /*CC2420SyncAlwaysOnC.AlarmC.Transform*/TransformAlarmC$1$Counter$overflow();
#line 70
  CC2420RadioM$Counter32khz$overflow();
#line 70
  /*Counter32khzC.CounterToLocalTimeC*/CounterToLocalTimeC$1$Counter$overflow();
#line 70
}
#line 70
# 112 "/Users/jingyuancheng/tinyos/moteiv/tos/lib/timer/TransformCounterC.nc"
static inline void /*Counter32khzC.Transform*/TransformCounterC$1$CounterFrom$overflow(void )
{
  /* atomic removed: atomic calls only */
  {
    /*Counter32khzC.Transform*/TransformCounterC$1$m_upper++;
    if ((/*Counter32khzC.Transform*/TransformCounterC$1$m_upper & /*Counter32khzC.Transform*/TransformCounterC$1$OVERFLOW_MASK) == 0) {
      /*Counter32khzC.Transform*/TransformCounterC$1$Counter$overflow();
      }
  }
}

# 65 "/Users/jingyuancheng/tinyos/moteiv/tos/lib/resource/ResourceCmdAsync.nc"
inline static void CC2420RadioM$CmdFlushRXFIFO$urgentRequest(uint8_t rh){
#line 65
  /*MSP430ArbiterUSART0C.ArbiterC.ArbiterP*/FcfsArbiterP$1$ResourceCmdAsync$urgentRequest(/*CC2420RadioC.CmdFlushRXFIFOC.ResourceC.ResourceUSARTP*/MSP430ResourceUSART0P$5$ID, rh);
#line 65
}
#line 65
# 188 "/Users/jingyuancheng/tinyos/moteiv/tos/lib/CC2420Radio/CC2420RadioM.nc"
static inline void CC2420RadioM$flushRXFIFO(uint8_t rh)
#line 188
{
  CC2420RadioM$CmdFlushRXFIFO$urgentRequest(rh);
}

# 30 "/Users/jingyuancheng/tinyos/moteiv/tos/platform/tmote/hardware.h"
static inline uint8_t TOSH_READ_CC_FIFOP_PIN()
#line 30
{
#line 30
  static volatile uint8_t r __asm ("0x0020");

#line 30
  return r & (1 << 0);
}

#line 31
static inline uint8_t TOSH_READ_CC_FIFO_PIN()
#line 31
{
#line 31
  static volatile uint8_t r __asm ("0x0020");

#line 31
  return r & (1 << 3);
}

# 913 "/Users/jingyuancheng/tinyos/moteiv/tos/lib/CC2420Radio/CC2420RadioM.nc"
static inline void CC2420RadioM$Counter32khz16$overflow(void )
#line 913
{
  /* atomic removed: atomic calls only */
#line 914
  {
    if (!TOSH_READ_CC_FIFO_PIN() && !TOSH_READ_CC_FIFOP_PIN()) {
        CC2420RadioM$flushRXFIFO(RESOURCE_NONE);
      }
  }
}

# 70 "/Users/jingyuancheng/tinyos/moteiv/tos/lib/timer/Counter.nc"
inline static void /*MSP430Counter32khzC.Counter*/MSP430CounterC$0$Counter$overflow(void ){
#line 70
  CC2420RadioM$Counter32khz16$overflow();
#line 70
  /*Counter32khzC.Transform*/TransformCounterC$1$CounterFrom$overflow();
#line 70
  /*CounterMilliC.Transform*/TransformCounterC$0$CounterFrom$overflow();
#line 70
}
#line 70
# 51 "/Users/jingyuancheng/tinyos/moteiv/tos/platform/msp430/timer/MSP430CounterC.nc"
static inline void /*MSP430Counter32khzC.Counter*/MSP430CounterC$0$MSP430Timer$overflow(void )
{
  /*MSP430Counter32khzC.Counter*/MSP430CounterC$0$Counter$overflow();
}

# 33 "/Users/jingyuancheng/tinyos/moteiv/tos/platform/msp430/timer/MSP430Timer.nc"
inline static void /*MSP430TimerC.MSP430TimerB*/MSP430TimerM$1$Timer$overflow(void ){
#line 33
  /*MSP430Counter32khzC.Counter*/MSP430CounterC$0$MSP430Timer$overflow();
#line 33
  MSP430DCOCalibP$TimerB$overflow();
#line 33
  /*HalTimerMilliC.AlarmMilliC.MSP430Alarm*/MSP430AlarmC$0$MSP430Timer$overflow();
#line 33
  /*CC2420RadioC.BackoffAlarm32khzC.MSP430Alarm*/MSP430AlarmC$1$MSP430Timer$overflow();
#line 33
  /*CC2420SyncAlwaysOnC.AlarmC.MSP430Alarm*/MSP430AlarmC$2$MSP430Timer$overflow();
#line 33
  /*MSP430TimerC.MSP430TimerB0*/MSP430TimerCapComM$3$Timer$overflow();
#line 33
  /*MSP430TimerC.MSP430TimerB1*/MSP430TimerCapComM$4$Timer$overflow();
#line 33
  /*MSP430TimerC.MSP430TimerB2*/MSP430TimerCapComM$5$Timer$overflow();
#line 33
  /*MSP430TimerC.MSP430TimerB3*/MSP430TimerCapComM$6$Timer$overflow();
#line 33
  /*MSP430TimerC.MSP430TimerB4*/MSP430TimerCapComM$7$Timer$overflow();
#line 33
  /*MSP430TimerC.MSP430TimerB5*/MSP430TimerCapComM$8$Timer$overflow();
#line 33
  /*MSP430TimerC.MSP430TimerB6*/MSP430TimerCapComM$9$Timer$overflow();
#line 33
}
#line 33
# 125 "/Users/jingyuancheng/tinyos/moteiv/tos/platform/msp430/timer/MSP430TimerM.nc"
static inline void /*MSP430TimerC.MSP430TimerB*/MSP430TimerM$1$Overflow$fired(void )
{
  /*MSP430TimerC.MSP430TimerB*/MSP430TimerM$1$Timer$overflow();
}

# 39 "/Users/jingyuancheng/tinyos/moteiv/tos/platform/tmote/hardware.h"
static inline void TOSH_SEL_UCLK0_MODFUNC()
#line 39
{
#line 39
  static volatile uint8_t r __asm ("0x001B");

#line 39
  r |= 1 << 3;
}

#line 37
static inline void TOSH_SEL_SOMI0_MODFUNC()
#line 37
{
#line 37
  static volatile uint8_t r __asm ("0x001B");

#line 37
  r |= 1 << 2;
}

#line 38
static inline void TOSH_SEL_SIMO0_MODFUNC()
#line 38
{
#line 38
  static volatile uint8_t r __asm ("0x001B");

#line 38
  r |= 1 << 1;
}


static inline void TOSH_SEL_URXD0_IOFUNC()
#line 42
{
#line 42
  static volatile uint8_t r __asm ("0x001B");

#line 42
  r &= ~(1 << 5);
}

#line 41
static inline void TOSH_SEL_UTXD0_IOFUNC()
#line 41
{
#line 41
  static volatile uint8_t r __asm ("0x001B");

#line 41
  r &= ~(1 << 4);
}

# 155 "/Users/jingyuancheng/tinyos/moteiv/tos/platform/msp430/HPLUSART0M.nc"
static inline void HPLUSART0M$USARTControl$disableUART(void )
#line 155
{
  HPLUSART0M$ME1 &= ~((1 << 7) | (1 << 6));
  TOSH_SEL_UTXD0_IOFUNC();
  TOSH_SEL_URXD0_IOFUNC();
}

#line 100
static inline bool HPLUSART0M$USARTControl$isI2C(void )
#line 100
{
  /* atomic removed: atomic calls only */
  {
    unsigned char __nesc_temp = (
#line 102
    U0CTL & ((0x20 | 0x04) | 0x01)) == ((0x20 | 0x04) | 0x01);

#line 102
    return __nesc_temp;
  }
}

#line 201
static inline void HPLUSART0M$USARTControl$disableI2C(void )
#line 201
{

  if (HPLUSART0M$USARTControl$isI2C()) {
    /* atomic removed: atomic calls only */
#line 204
    U0CTL &= ~((0x20 | 0x01) | 0x04);
    }
}

static inline void HPLUSART0M$USARTControl$setModeSPI(void )
#line 208
{
  /* atomic removed: atomic calls only */
#line 209
  {



    HPLUSART0M$USARTControl$disableI2C();
    U0CTL = 0x01;

    HPLUSART0M$USARTControl$disableUART();

    TOSH_SEL_SIMO0_MODFUNC();
    TOSH_SEL_SOMI0_MODFUNC();
    TOSH_SEL_UCLK0_MODFUNC();

    HPLUSART0M$IE1 &= ~((1 << 7) | (1 << 6));

    U0CTL = 0x01;
    U0CTL |= (0x10 | 0x04) | 0x02;
    U0CTL &= ~0x20;

    HPLUSART0M$U0TCTL = 0x02;
    HPLUSART0M$U0TCTL |= 0x80;

    if (HPLUSART0M$l_ssel & 0x80) {
        HPLUSART0M$U0TCTL &= ~(((0x00 | 0x10) | 0x20) | 0x30);
        HPLUSART0M$U0TCTL |= HPLUSART0M$l_ssel & 0x7F;
      }
    else {
        HPLUSART0M$U0TCTL &= ~(((0x00 | 0x10) | 0x20) | 0x30);
        HPLUSART0M$U0TCTL |= 0x20;
      }

    if (HPLUSART0M$l_br != 0) {
        U0BR0 = HPLUSART0M$l_br & 0x0FF;
        U0BR1 = (HPLUSART0M$l_br >> 8) & 0x0FF;
      }
    else {
        U0BR0 = 0x02;
        U0BR1 = 0x00;
      }
    U0MCTL = 0;

    HPLUSART0M$ME1 &= ~((1 << 7) | (1 << 6));
    HPLUSART0M$ME1 |= 1 << 6;
    U0CTL &= ~0x01;

    HPLUSART0M$IFG1 &= ~((1 << 7) | (1 << 6));
    HPLUSART0M$IE1 &= ~((1 << 7) | (1 << 6));
  }
}

# 135 "/Users/jingyuancheng/tinyos/moteiv/tinyos-1.x/tos/platform/msp430/HPLUSARTControl.nc"
inline static void /*MSP430ResourceConfigUSART0C.ConfigUSARTP*/MSP430ResourceConfigUSARTP$0$HPLUSARTControl$setModeSPI(void ){
#line 135
  HPLUSART0M$USARTControl$setModeSPI();
#line 135
}
#line 135
# 434 "/Users/jingyuancheng/tinyos/moteiv/tos/platform/msp430/HPLUSART0M.nc"
static inline result_t HPLUSART0M$USARTControl$disableRxIntr(void )
#line 434
{
  HPLUSART0M$IE1 &= ~(1 << 6);
  return SUCCESS;
}

# 172 "/Users/jingyuancheng/tinyos/moteiv/tinyos-1.x/tos/platform/msp430/HPLUSARTControl.nc"
inline static result_t /*MSP430ResourceConfigUSART0C.ConfigUSARTP*/MSP430ResourceConfigUSARTP$0$HPLUSARTControl$disableRxIntr(void ){
#line 172
  unsigned char __nesc_result;
#line 172

#line 172
  __nesc_result = HPLUSART0M$USARTControl$disableRxIntr();
#line 172

#line 172
  return __nesc_result;
#line 172
}
#line 172
# 439 "/Users/jingyuancheng/tinyos/moteiv/tos/platform/msp430/HPLUSART0M.nc"
static inline result_t HPLUSART0M$USARTControl$disableTxIntr(void )
#line 439
{
  HPLUSART0M$IE1 &= ~(1 << 7);
  return SUCCESS;
}

# 173 "/Users/jingyuancheng/tinyos/moteiv/tinyos-1.x/tos/platform/msp430/HPLUSARTControl.nc"
inline static result_t /*MSP430ResourceConfigUSART0C.ConfigUSARTP*/MSP430ResourceConfigUSARTP$0$HPLUSARTControl$disableTxIntr(void ){
#line 173
  unsigned char __nesc_result;
#line 173

#line 173
  __nesc_result = HPLUSART0M$USARTControl$disableTxIntr();
#line 173

#line 173
  return __nesc_result;
#line 173
}
#line 173
# 134 "/Users/jingyuancheng/tinyos/moteiv/tos/lib/util/reservedQueue.h"
static inline bool rqueue_pushFront(ReservedQueue_t *q, uint8_t id)
#line 134
{
  if (rqueue_isQueued(q, id)) {
      return FALSE;
    }

  q->next[id] = q->head;
  q->head = id;
  if (q->tail == RQUEUE_NONE) {
    q->tail = id;
    }
#line 143
  return TRUE;
}

# 49 "/Users/jingyuancheng/tinyos/moteiv/tos/lib/CC2420Radio/HPLCC2420RAM.nc"
inline static result_t CC2420TimeStampingM$HPLCC2420RAM$write(uint8_t rh, uint16_t addr, uint8_t length, uint8_t *buffer){
#line 49
  unsigned char __nesc_result;
#line 49

#line 49
  __nesc_result = HPLCC2420M$HPLCC2420RAM$write(rh, addr, length, buffer);
#line 49

#line 49
  return __nesc_result;
#line 49
}
#line 49
# 76 "/Users/jingyuancheng/tinyos/moteiv/tos/lib/resource/FcfsArbiterP.nc"
static inline bool /*MSP430ArbiterUSART0C.ArbiterC.ArbiterP*/FcfsArbiterP$1$ResourceValidate$validateUser(uint8_t rh)
#line 76
{
  /* atomic removed: atomic calls only */
#line 77
  {
    unsigned char __nesc_temp = 
#line 77
    rh != RESOURCE_NONE && rh == /*MSP430ArbiterUSART0C.ArbiterC.ArbiterP*/FcfsArbiterP$1$m_granted;

#line 77
    return __nesc_temp;
  }
}

# 28 "/Users/jingyuancheng/tinyos/moteiv/tos/lib/resource/ResourceValidate.nc"
inline static bool HPLCC2420M$CC2420Validate$validateUser(uint8_t rh){
#line 28
  unsigned char __nesc_result;
#line 28

#line 28
  __nesc_result = /*MSP430ArbiterUSART0C.ArbiterC.ArbiterP*/FcfsArbiterP$1$ResourceValidate$validateUser(rh);
#line 28

#line 28
  return __nesc_result;
#line 28
}
#line 28
# 191 "/Users/jingyuancheng/tinyos/moteiv/tos/lib/sched/SchedulerBasicP.nc"
static inline result_t SchedulerBasicP$TaskBasic$postTask(uint8_t id)
#line 191
{
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 192
    SchedulerBasicP$pushTask(id);
#line 192
    __nesc_atomic_end(__nesc_atomic); }
  return SUCCESS;
}

# 49 "/Users/jingyuancheng/tinyos/moteiv/tos/lib/sched/TaskBasic.nc"
inline static result_t HPLCC2420M$signalRAMWr$postTask(void ){
#line 49
  unsigned char __nesc_result;
#line 49

#line 49
  __nesc_result = SchedulerBasicP$TaskBasic$postTask(HPLCC2420M$signalRAMWr);
#line 49

#line 49
  return __nesc_result;
#line 49
}
#line 49
# 219 "/Users/jingyuancheng/tinyos/moteiv/tos/lib/resource/FcfsArbiterP.nc"
static inline void /*MSP430ArbiterUSART0C.ArbiterC.ArbiterP*/FcfsArbiterP$1$ResourceCmdAsync$release(uint8_t id)
#line 219
{
  /*MSP430ArbiterUSART0C.ArbiterC.ArbiterP*/FcfsArbiterP$1$Resource$release(id);
}

# 85 "/Users/jingyuancheng/tinyos/moteiv/tos/lib/resource/ResourceCmdAsync.nc"
inline static void CC2420TimeStampingM$CmdWriteTimeStamp$release(void ){
#line 85
  /*MSP430ArbiterUSART0C.ArbiterC.ArbiterP*/FcfsArbiterP$1$ResourceCmdAsync$release(/*CC2420TimeStampingC.CmdWriteTimeStampC.ResourceC.ResourceUSARTP*/MSP430ResourceUSART0P$9$ID);
#line 85
}
#line 85
# 92 "/Users/jingyuancheng/tinyos/moteiv/tos/lib/util/reservedQueue.h"
static inline uint8_t rqueue_pop(ReservedQueue_t *q)
#line 92
{
  uint8_t head = q->head;

#line 94
  rqueue_remove(q, head);
  return head;
}

# 196 "/Users/jingyuancheng/tinyos/moteiv/tos/lib/sched/SchedulerBasicP.nc"
static inline result_t SchedulerBasicP$TaskBasic$postUrgentTask(uint8_t id)
#line 196
{
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 197
    SchedulerBasicP$pushFront(id);
#line 197
    __nesc_atomic_end(__nesc_atomic); }
  return SUCCESS;
}

# 50 "/Users/jingyuancheng/tinyos/moteiv/tos/lib/sched/TaskBasic.nc"
inline static result_t /*MSP430ArbiterUSART0C.ArbiterC.ArbiterP*/FcfsArbiterP$1$GrantTask$postUrgentTask(void ){
#line 50
  unsigned char __nesc_result;
#line 50

#line 50
  __nesc_result = SchedulerBasicP$TaskBasic$postUrgentTask(19U);
#line 50

#line 50
  return __nesc_result;
#line 50
}
#line 50
# 96 "/Users/jingyuancheng/tinyos/moteiv/tos/lib/sched/SchedulerBasicP.nc"
static inline bool SchedulerBasicP$isQueued(uint8_t id)
#line 96
{
  return SchedulerBasicP$m_next[id] != SchedulerBasicP$NONE || SchedulerBasicP$m_tail == id;
}

# 85 "/Users/jingyuancheng/tinyos/moteiv/tinyos-1.x/tos/platform/msp430/HPLUSARTControl.nc"
inline static void /*MSP430ResourceConfigUSART0C.ConfigUSARTP*/MSP430ResourceConfigUSARTP$0$HPLUSARTControl$disableUART(void ){
#line 85
  HPLUSART0M$USARTControl$disableUART();
#line 85
}
#line 85
# 39 "/Users/jingyuancheng/tinyos/moteiv/tos/platform/tmote/hardware.h"
static inline void TOSH_SEL_UCLK0_IOFUNC()
#line 39
{
#line 39
  static volatile uint8_t r __asm ("0x001B");

#line 39
  r &= ~(1 << 3);
}

#line 37
static inline void TOSH_SEL_SOMI0_IOFUNC()
#line 37
{
#line 37
  static volatile uint8_t r __asm ("0x001B");

#line 37
  r &= ~(1 << 2);
}

#line 38
static inline void TOSH_SEL_SIMO0_IOFUNC()
#line 38
{
#line 38
  static volatile uint8_t r __asm ("0x001B");

#line 38
  r &= ~(1 << 1);
}

# 188 "/Users/jingyuancheng/tinyos/moteiv/tos/platform/msp430/HPLUSART0M.nc"
static inline void HPLUSART0M$USARTControl$disableSPI(void )
#line 188
{
  HPLUSART0M$ME1 &= ~(1 << 6);
  TOSH_SEL_SIMO0_IOFUNC();
  TOSH_SEL_SOMI0_IOFUNC();
  TOSH_SEL_UCLK0_IOFUNC();
}

# 115 "/Users/jingyuancheng/tinyos/moteiv/tinyos-1.x/tos/platform/msp430/HPLUSARTControl.nc"
inline static void /*MSP430ResourceConfigUSART0C.ConfigUSARTP*/MSP430ResourceConfigUSARTP$0$HPLUSARTControl$disableSPI(void ){
#line 115
  HPLUSART0M$USARTControl$disableSPI();
#line 115
}
#line 115










inline static void /*MSP430ResourceConfigUSART0C.ConfigUSARTP*/MSP430ResourceConfigUSARTP$0$HPLUSARTControl$disableI2C(void ){
#line 125
  HPLUSART0M$USARTControl$disableI2C();
#line 125
}
#line 125
# 71 "/Users/jingyuancheng/tinyos/moteiv/tos/platform/msp430/resource/MSP430ResourceConfigUSARTP.nc"
static inline void /*MSP430ResourceConfigUSART0C.ConfigUSARTP*/MSP430ResourceConfigUSARTP$0$Arbiter$idle(void )
#line 71
{
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 72
    {
      /*MSP430ResourceConfigUSART0C.ConfigUSARTP*/MSP430ResourceConfigUSARTP$0$m_mode = /*MSP430ResourceConfigUSART0C.ConfigUSARTP*/MSP430ResourceConfigUSARTP$0$MODE_UNKNOWN;
      /*MSP430ResourceConfigUSART0C.ConfigUSARTP*/MSP430ResourceConfigUSARTP$0$HPLUSARTControl$disableI2C();
      /*MSP430ResourceConfigUSART0C.ConfigUSARTP*/MSP430ResourceConfigUSARTP$0$HPLUSARTControl$disableSPI();
      /*MSP430ResourceConfigUSART0C.ConfigUSARTP*/MSP430ResourceConfigUSARTP$0$HPLUSARTControl$disableUART();
    }
#line 77
    __nesc_atomic_end(__nesc_atomic); }
}

# 60 "/Users/jingyuancheng/tinyos/moteiv/tos/lib/resource/Arbiter.nc"
inline static void /*MSP430ArbiterUSART0C.ArbiterC.ArbiterP*/FcfsArbiterP$1$Arbiter$idle(void ){
#line 60
  /*MSP430ResourceConfigUSART0C.ConfigUSARTP*/MSP430ResourceConfigUSARTP$0$Arbiter$idle();
#line 60
}
#line 60
# 71 "/Users/jingyuancheng/tinyos/moteiv/tos/lib/CC2420Radio/HPLCC2420.nc"
inline static uint16_t CC2420RadioM$HPLChipcon$read(uint8_t rh, uint8_t addr){
#line 71
  unsigned int __nesc_result;
#line 71

#line 71
  __nesc_result = HPLCC2420M$HPLCC2420$read(rh, addr);
#line 71

#line 71
  return __nesc_result;
#line 71
}
#line 71
# 83 "/Users/jingyuancheng/tinyos/moteiv/tos/platform/tmote/HPLCC2420M.nc"
static inline uint8_t HPLCC2420M$adjustStatusByte(uint8_t status)
#line 83
{
  return status & 0x7E;
}

# 32 "/Users/jingyuancheng/tinyos/moteiv/tos/platform/tmote/hardware.h"
static inline void TOSH_SEL_CC_SFD_MODFUNC()
#line 32
{
#line 32
  static volatile uint8_t r __asm ("0x001F");

#line 32
  r |= 1 << 1;
}

# 45 "/Users/jingyuancheng/tinyos/moteiv/tos/platform/msp430/timer/MSP430TimerCapComM.nc"
static inline uint16_t /*MSP430TimerC.MSP430TimerB1*/MSP430TimerCapComM$4$CC2int(/*MSP430TimerC.MSP430TimerB1*/MSP430TimerCapComM$4$CC_t x)
#line 45
{
#line 45
  union /*MSP430TimerC.MSP430TimerB1*/MSP430TimerCapComM$4$__nesc_unnamed4388 {
#line 45
    /*MSP430TimerC.MSP430TimerB1*/MSP430TimerCapComM$4$CC_t f;
#line 45
    uint16_t t;
  } 
#line 45
  c = { .f = x };

#line 45
  return c.t;
}

#line 60
static inline uint16_t /*MSP430TimerC.MSP430TimerB1*/MSP430TimerCapComM$4$captureControl(uint8_t l_cm)
{
  /*MSP430TimerC.MSP430TimerB1*/MSP430TimerCapComM$4$CC_t x = { 
  .cm = l_cm & 0x03, 
  .ccis = 0, 
  .clld = 0, 
  .cap = 1, 
  .scs = 1, 
  .ccie = 0 };

  return /*MSP430TimerC.MSP430TimerB1*/MSP430TimerCapComM$4$CC2int(x);
}

#line 98
static inline void /*MSP430TimerC.MSP430TimerB1*/MSP430TimerCapComM$4$Control$setControlAsCapture(uint8_t capture_mode)
{
  * (volatile uint16_t *)388U = /*MSP430TimerC.MSP430TimerB1*/MSP430TimerCapComM$4$captureControl(capture_mode);
}

# 36 "/Users/jingyuancheng/tinyos/moteiv/tos/platform/msp430/timer/MSP430TimerControl.nc"
inline static void HPLCC2420InterruptM$SFDControl$setControlAsCapture(uint8_t capture_mode){
#line 36
  /*MSP430TimerC.MSP430TimerB1*/MSP430TimerCapComM$4$Control$setControlAsCapture(capture_mode);
#line 36
}
#line 36
# 118 "/Users/jingyuancheng/tinyos/moteiv/tos/platform/msp430/timer/MSP430TimerCapComM.nc"
static inline void /*MSP430TimerC.MSP430TimerB1*/MSP430TimerCapComM$4$Control$enableEvents(void )
{
  * (volatile uint16_t *)388U |= 0x0010;
}

# 38 "/Users/jingyuancheng/tinyos/moteiv/tos/platform/msp430/timer/MSP430TimerControl.nc"
inline static void HPLCC2420InterruptM$SFDControl$enableEvents(void ){
#line 38
  /*MSP430TimerC.MSP430TimerB1*/MSP430TimerCapComM$4$Control$enableEvents();
#line 38
}
#line 38
# 225 "/Users/jingyuancheng/tinyos/moteiv/tos/platform/msp430/MSP430InterruptM.nc"
static inline void MSP430InterruptM$Port10$edge(bool l2h)
#line 225
{
  /* atomic removed: atomic calls only */
#line 226
  {
    if (l2h) {
#line 227
      P1IES &= ~(1 << 0);
      }
    else {
#line 228
      P1IES |= 1 << 0;
      }
  }
}

# 58 "/Users/jingyuancheng/tinyos/moteiv/tos/platform/msp430/MSP430Interrupt.nc"
inline static void HPLCC2420InterruptM$FIFOPInterrupt$edge(bool low_to_high){
#line 58
  MSP430InterruptM$Port10$edge(low_to_high);
#line 58
}
#line 58
# 118 "/Users/jingyuancheng/tinyos/moteiv/tos/platform/msp430/MSP430InterruptM.nc"
static inline void MSP430InterruptM$Port10$enable(void )
#line 118
{
#line 118
  MSP430InterruptM$P1IE |= 1 << 0;
}

# 34 "/Users/jingyuancheng/tinyos/moteiv/tos/platform/msp430/MSP430Interrupt.nc"
inline static void HPLCC2420InterruptM$FIFOPInterrupt$enable(void ){
#line 34
  MSP430InterruptM$Port10$enable();
#line 34
}
#line 34
# 85 "/Users/jingyuancheng/tinyos/moteiv/tos/lib/resource/ResourceCmdAsync.nc"
inline static void CC2420RadioM$CmdFlushRXFIFO$release(void ){
#line 85
  /*MSP430ArbiterUSART0C.ArbiterC.ArbiterP*/FcfsArbiterP$1$ResourceCmdAsync$release(/*CC2420RadioC.CmdFlushRXFIFOC.ResourceC.ResourceUSARTP*/MSP430ResourceUSART0P$5$ID);
#line 85
}
#line 85
# 48 "/Users/jingyuancheng/tinyos/moteiv/tos/lib/CC2420Radio/HPLCC2420.nc"
inline static uint8_t CC2420ControlM$HPLChipcon$cmd(uint8_t rh, uint8_t addr){
#line 48
  unsigned char __nesc_result;
#line 48

#line 48
  __nesc_result = HPLCC2420M$HPLCC2420$cmd(rh, addr);
#line 48

#line 48
  return __nesc_result;
#line 48
}
#line 48
#line 60
inline static uint8_t CC2420ControlM$HPLChipcon$write(uint8_t rh, uint8_t addr, uint16_t data){
#line 60
  unsigned char __nesc_result;
#line 60

#line 60
  __nesc_result = HPLCC2420M$HPLCC2420$write(rh, addr, data);
#line 60

#line 60
  return __nesc_result;
#line 60
}
#line 60
# 270 "/Users/jingyuancheng/tinyos/moteiv/tos/lib/CC2420Radio/CC2420ControlM.nc"
static inline void CC2420ControlM$doCmdFreqSelect(uint8_t rh)
#line 270
{
  uint8_t status = CC2420ControlM$HPLChipcon$write(rh, 0x18, CC2420ControlM$gCurrentParameters[CP_FSCTRL]);


  if (status & (1 << 6)) {
    CC2420ControlM$HPLChipcon$cmd(rh, 0x03);
    }
}

#line 374
static inline void CC2420ControlM$doCmdSetRFPower(uint8_t rh)
#line 374
{
  CC2420ControlM$HPLChipcon$write(rh, 0x15, CC2420ControlM$gCurrentParameters[CP_TXCTRL]);
}

# 49 "/Users/jingyuancheng/tinyos/moteiv/tos/lib/CC2420Radio/HPLCC2420RAM.nc"
inline static result_t CC2420ControlM$HPLChipconRAM$write(uint8_t rh, uint16_t addr, uint8_t length, uint8_t *buffer){
#line 49
  unsigned char __nesc_result;
#line 49

#line 49
  __nesc_result = HPLCC2420M$HPLCC2420RAM$write(rh, addr, length, buffer);
#line 49

#line 49
  return __nesc_result;
#line 49
}
#line 49
# 474 "/Users/jingyuancheng/tinyos/moteiv/tos/lib/CC2420Radio/CC2420ControlM.nc"
static inline void CC2420ControlM$doCmdSetShortAddress(uint8_t rh)
#line 474
{
  CC2420ControlM$HPLChipconRAM$write(rh, 0x16A, 2, (uint8_t *)&CC2420ControlM$shortAddress);
}

#line 442
static inline void CC2420ControlM$doCmdMDMCTRL0(uint8_t rh)
#line 442
{
  CC2420ControlM$HPLChipcon$write(rh, 0x11, CC2420ControlM$gCurrentParameters[CP_MDMCTRL0]);
}

# 122 "/Users/jingyuancheng/tinyos/moteiv/tos/platform/msp430/MSP430InterruptM.nc"
static inline void MSP430InterruptM$Port14$enable(void )
#line 122
{
#line 122
  MSP430InterruptM$P1IE |= 1 << 4;
}

# 34 "/Users/jingyuancheng/tinyos/moteiv/tos/platform/msp430/MSP430Interrupt.nc"
inline static void HPLCC2420InterruptM$CCAInterrupt$enable(void ){
#line 34
  MSP430InterruptM$Port14$enable();
#line 34
}
#line 34
# 249 "/Users/jingyuancheng/tinyos/moteiv/tos/platform/msp430/MSP430InterruptM.nc"
static inline void MSP430InterruptM$Port14$edge(bool l2h)
#line 249
{
  /* atomic removed: atomic calls only */
#line 250
  {
    if (l2h) {
#line 251
      P1IES &= ~(1 << 4);
      }
    else {
#line 252
      P1IES |= 1 << 4;
      }
  }
}

# 58 "/Users/jingyuancheng/tinyos/moteiv/tos/platform/msp430/MSP430Interrupt.nc"
inline static void HPLCC2420InterruptM$CCAInterrupt$edge(bool low_to_high){
#line 58
  MSP430InterruptM$Port14$edge(low_to_high);
#line 58
}
#line 58
# 184 "/Users/jingyuancheng/tinyos/moteiv/tos/platform/msp430/MSP430InterruptM.nc"
static inline void MSP430InterruptM$Port14$clear(void )
#line 184
{
#line 184
  MSP430InterruptM$P1IFG &= ~(1 << 4);
}

# 44 "/Users/jingyuancheng/tinyos/moteiv/tos/platform/msp430/MSP430Interrupt.nc"
inline static void HPLCC2420InterruptM$CCAInterrupt$clear(void ){
#line 44
  MSP430InterruptM$Port14$clear();
#line 44
}
#line 44
# 153 "/Users/jingyuancheng/tinyos/moteiv/tos/platform/msp430/MSP430InterruptM.nc"
static inline void MSP430InterruptM$Port14$disable(void )
#line 153
{
#line 153
  MSP430InterruptM$P1IE &= ~(1 << 4);
}

# 39 "/Users/jingyuancheng/tinyos/moteiv/tos/platform/msp430/MSP430Interrupt.nc"
inline static void HPLCC2420InterruptM$CCAInterrupt$disable(void ){
#line 39
  MSP430InterruptM$Port14$disable();
#line 39
}
#line 39
# 155 "/Users/jingyuancheng/tinyos/moteiv/tos/platform/tmote/HPLCC2420InterruptM.nc"
static inline result_t HPLCC2420InterruptM$CCA$startWait(bool low_to_high)
#line 155
{
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 156
    {
      HPLCC2420InterruptM$CCAInterrupt$disable();
      HPLCC2420InterruptM$CCAInterrupt$clear();
      HPLCC2420InterruptM$CCAInterrupt$edge(low_to_high);
      HPLCC2420InterruptM$CCAInterrupt$enable();
    }
#line 161
    __nesc_atomic_end(__nesc_atomic); }
  return SUCCESS;
}

# 43 "/Users/jingyuancheng/tinyos/moteiv/tinyos-1.x/tos/lib/CC2420Radio/HPLCC2420Interrupt.nc"
inline static result_t CC2420ControlM$CCA$startWait(bool low_to_high){
#line 43
  unsigned char __nesc_result;
#line 43

#line 43
  __nesc_result = HPLCC2420InterruptM$CCA$startWait(low_to_high);
#line 43

#line 43
  return __nesc_result;
#line 43
}
#line 43
# 393 "/Users/jingyuancheng/tinyos/moteiv/tos/lib/CC2420Radio/CC2420ControlM.nc"
static inline void CC2420ControlM$doCmdOscillatorOn(uint8_t rh)
#line 393
{










  CC2420ControlM$HPLChipcon$write(rh, 0x1D, 24);


  CC2420ControlM$CCA$startWait(TRUE);


  CC2420ControlM$HPLChipcon$cmd(rh, 0x01);
}








static inline void CC2420ControlM$doCmdOscillatorOff(uint8_t rh)
#line 420
{
  CC2420ControlM$HPLChipcon$cmd(rh, 0x07);
}

#line 358
static inline void CC2420ControlM$doCmdSRXON(uint8_t rh)
#line 358
{
  CC2420ControlM$HPLChipcon$cmd(rh, 0x03);
}

#line 327
static inline void CC2420ControlM$doCmdSTXON(uint8_t rh)
#line 327
{
  CC2420ControlM$HPLChipcon$cmd(rh, 0x04);
}

#line 344
static inline void CC2420ControlM$doCmdSTXONCCA(uint8_t rh)
#line 344
{
  CC2420ControlM$HPLChipcon$cmd(rh, 0x05);
}

# 85 "/Users/jingyuancheng/tinyos/moteiv/tos/lib/resource/ResourceCmdAsync.nc"
inline static void CC2420ControlM$CmdCmds$release(void ){
#line 85
  /*MSP430ArbiterUSART0C.ArbiterC.ArbiterP*/FcfsArbiterP$1$ResourceCmdAsync$release(/*CC2420RadioC.CmdCmds.ResourceC.ResourceUSARTP*/MSP430ResourceUSART0P$4$ID);
#line 85
}
#line 85
# 80 "/Users/jingyuancheng/tinyos/moteiv/tos/platform/msp430/resource/MSP430ResourceConfigUSARTP.nc"
static inline void /*MSP430ResourceConfigUSART0C.ConfigUSARTP*/MSP430ResourceConfigUSARTP$0$Arbiter$requested(void )
#line 80
{
}

# 54 "/Users/jingyuancheng/tinyos/moteiv/tos/lib/resource/Arbiter.nc"
inline static void /*MSP430ArbiterUSART0C.ArbiterC.ArbiterP*/FcfsArbiterP$1$Arbiter$requested(void ){
#line 54
  /*MSP430ResourceConfigUSART0C.ConfigUSARTP*/MSP430ResourceConfigUSARTP$0$Arbiter$requested();
#line 54
}
#line 54
# 40 "/Users/jingyuancheng/tinyos/moteiv/tos/platform/msp430/timer/MSP430DCOCalibP.nc"
static inline void MSP430DCOCalibP$TimerCompareB$fired(void )
#line 40
{
}

# 34 "/Users/jingyuancheng/tinyos/moteiv/tos/platform/msp430/timer/MSP430Compare.nc"
inline static void /*MSP430TimerC.MSP430TimerB0*/MSP430TimerCapComM$3$Compare$fired(void ){
#line 34
  MSP430DCOCalibP$TimerCompareB$fired();
#line 34
}
#line 34
# 138 "/Users/jingyuancheng/tinyos/moteiv/tos/platform/msp430/timer/MSP430TimerCapComM.nc"
static inline uint16_t /*MSP430TimerC.MSP430TimerB0*/MSP430TimerCapComM$3$Capture$getEvent(void )
{
  return * (volatile uint16_t *)402U;
}

#line 176
static inline void /*MSP430TimerC.MSP430TimerB0*/MSP430TimerCapComM$3$Capture$default$captured(uint16_t n)
{
}

# 74 "/Users/jingyuancheng/tinyos/moteiv/tos/platform/msp430/timer/MSP430Capture.nc"
inline static void /*MSP430TimerC.MSP430TimerB0*/MSP430TimerCapComM$3$Capture$captured(uint16_t time){
#line 74
  /*MSP430TimerC.MSP430TimerB0*/MSP430TimerCapComM$3$Capture$default$captured(time);
#line 74
}
#line 74
# 46 "/Users/jingyuancheng/tinyos/moteiv/tos/platform/msp430/timer/MSP430TimerCapComM.nc"
static inline /*MSP430TimerC.MSP430TimerB0*/MSP430TimerCapComM$3$CC_t /*MSP430TimerC.MSP430TimerB0*/MSP430TimerCapComM$3$int2CC(uint16_t x)
#line 46
{
#line 46
  union /*MSP430TimerC.MSP430TimerB0*/MSP430TimerCapComM$3$__nesc_unnamed4389 {
#line 46
    uint16_t f;
#line 46
    /*MSP430TimerC.MSP430TimerB0*/MSP430TimerCapComM$3$CC_t t;
  } 
#line 46
  c = { .f = x };

#line 46
  return c.t;
}

#line 73
static inline /*MSP430TimerC.MSP430TimerB0*/MSP430TimerCapComM$3$CC_t /*MSP430TimerC.MSP430TimerB0*/MSP430TimerCapComM$3$Control$getControl(void )
{
  return /*MSP430TimerC.MSP430TimerB0*/MSP430TimerCapComM$3$int2CC(* (volatile uint16_t *)386U);
}

#line 168
static inline void /*MSP430TimerC.MSP430TimerB0*/MSP430TimerCapComM$3$Event$fired(void )
{
  if (/*MSP430TimerC.MSP430TimerB0*/MSP430TimerCapComM$3$Control$getControl().cap) {
    /*MSP430TimerC.MSP430TimerB0*/MSP430TimerCapComM$3$Capture$captured(/*MSP430TimerC.MSP430TimerB0*/MSP430TimerCapComM$3$Capture$getEvent());
    }
  else {
#line 173
    /*MSP430TimerC.MSP430TimerB0*/MSP430TimerCapComM$3$Compare$fired();
    }
}




static inline void /*MSP430TimerC.MSP430TimerB1*/MSP430TimerCapComM$4$Compare$default$fired(void )
{
}

# 34 "/Users/jingyuancheng/tinyos/moteiv/tos/platform/msp430/timer/MSP430Compare.nc"
inline static void /*MSP430TimerC.MSP430TimerB1*/MSP430TimerCapComM$4$Compare$fired(void ){
#line 34
  /*MSP430TimerC.MSP430TimerB1*/MSP430TimerCapComM$4$Compare$default$fired();
#line 34
}
#line 34
# 138 "/Users/jingyuancheng/tinyos/moteiv/tos/platform/msp430/timer/MSP430TimerCapComM.nc"
static inline uint16_t /*MSP430TimerC.MSP430TimerB1*/MSP430TimerCapComM$4$Capture$getEvent(void )
{
  return * (volatile uint16_t *)404U;
}

#line 163
static inline void /*MSP430TimerC.MSP430TimerB1*/MSP430TimerCapComM$4$Capture$clearOverflow(void )
{
  * (volatile uint16_t *)388U &= ~0x0002;
}

# 56 "/Users/jingyuancheng/tinyos/moteiv/tos/platform/msp430/timer/MSP430Capture.nc"
inline static void HPLCC2420InterruptM$SFDCapture$clearOverflow(void ){
#line 56
  /*MSP430TimerC.MSP430TimerB1*/MSP430TimerCapComM$4$Capture$clearOverflow();
#line 56
}
#line 56
# 158 "/Users/jingyuancheng/tinyos/moteiv/tos/platform/msp430/timer/MSP430TimerCapComM.nc"
static inline bool /*MSP430TimerC.MSP430TimerB1*/MSP430TimerCapComM$4$Capture$isOverflowPending(void )
{
  return * (volatile uint16_t *)388U & 0x0002;
}

# 51 "/Users/jingyuancheng/tinyos/moteiv/tos/platform/msp430/timer/MSP430Capture.nc"
inline static bool HPLCC2420InterruptM$SFDCapture$isOverflowPending(void ){
#line 51
  unsigned char __nesc_result;
#line 51

#line 51
  __nesc_result = /*MSP430TimerC.MSP430TimerB1*/MSP430TimerCapComM$4$Capture$isOverflowPending();
#line 51

#line 51
  return __nesc_result;
#line 51
}
#line 51
# 83 "/Users/jingyuancheng/tinyos/moteiv/tos/platform/msp430/timer/MSP430TimerCapComM.nc"
static inline void /*MSP430TimerC.MSP430TimerB1*/MSP430TimerCapComM$4$Control$clearPendingInterrupt(void )
{
  * (volatile uint16_t *)388U &= ~0x0001;
}

# 32 "/Users/jingyuancheng/tinyos/moteiv/tos/platform/msp430/timer/MSP430TimerControl.nc"
inline static void HPLCC2420InterruptM$SFDControl$clearPendingInterrupt(void ){
#line 32
  /*MSP430TimerC.MSP430TimerB1*/MSP430TimerCapComM$4$Control$clearPendingInterrupt();
#line 32
}
#line 32
# 123 "/Users/jingyuancheng/tinyos/moteiv/tos/platform/msp430/timer/MSP430TimerCapComM.nc"
static inline void /*MSP430TimerC.MSP430TimerB1*/MSP430TimerCapComM$4$Control$disableEvents(void )
{
  * (volatile uint16_t *)388U &= ~0x0010;
}

# 39 "/Users/jingyuancheng/tinyos/moteiv/tos/platform/msp430/timer/MSP430TimerControl.nc"
inline static void HPLCC2420InterruptM$SFDControl$disableEvents(void ){
#line 39
  /*MSP430TimerC.MSP430TimerB1*/MSP430TimerCapComM$4$Control$disableEvents();
#line 39
}
#line 39
# 71 "/Users/jingyuancheng/tinyos/moteiv/tos/lib/sp/cc2420/CC2420TimeStampingM.nc"
static inline void CC2420TimeStampingM$RadioReceiveCoordinator$startSymbol(uint8_t bitsPerBlock, uint8_t offset, TOS_MsgPtr msgBuff)
{
}

# 33 "/Users/jingyuancheng/tinyos/moteiv/tinyos-1.x/tos/interfaces/RadioCoordinator.nc"
inline static void CC2420RadioM$RadioReceiveCoordinator$startSymbol(uint8_t bitsPerBlock, uint8_t offset, TOS_MsgPtr msgBuff){
#line 33
  CC2420TimeStampingM$RadioReceiveCoordinator$startSymbol(bitsPerBlock, offset, msgBuff);
#line 33
}
#line 33
# 63 "/Users/jingyuancheng/tinyos/moteiv/tos/lib/util/circularQueue.h"
static inline CircularQueueIndex_t cqueue_privDec(CircularQueue_t *cq, CircularQueueIndex_t n)
{
  return n ? n - 1 : cq->size - 1;
}

#line 114
static inline result_t cqueue_pushBack(CircularQueue_t *cq)
{
  if (cqueue_isEmpty(cq) == TRUE) 
    {
      cq->front = 0;
      cq->back = 0;
    }
  else 
    {
      CircularQueueIndex_t newback = cqueue_privDec(cq, cq->back);

      if (newback == cq->front) {
        return FAIL;
        }
      cq->back = newback;
    }

  return SUCCESS;
}

# 63 "/Users/jingyuancheng/tinyos/moteiv/tinyos-1.x/tos/interfaces/Random.nc"
inline static uint16_t CC2420AlwaysOnM$Random$rand(void ){
#line 63
  unsigned int __nesc_result;
#line 63

#line 63
  __nesc_result = RandomMLCG$Random$rand();
#line 63

#line 63
  return __nesc_result;
#line 63
}
#line 63
# 173 "/Users/jingyuancheng/tinyos/moteiv/tos/lib/sp/cc2420/CC2420AlwaysOnM.nc"
static inline int16_t CC2420AlwaysOnM$MacBackoff$congestionBackoff(TOS_MsgPtr m)
#line 173
{
  CC2420AlwaysOnM$m_backoffs++;
  return (CC2420AlwaysOnM$Random$rand() & 0x7F) + 1;
}

# 20 "/Users/jingyuancheng/tinyos/moteiv/tos/lib/CC2420Radio/MacBackoff.nc"
inline static int16_t CC2420RadioM$MacBackoff$congestionBackoff(TOS_MsgPtr m){
#line 20
  int __nesc_result;
#line 20

#line 20
  __nesc_result = CC2420AlwaysOnM$MacBackoff$congestionBackoff(m);
#line 20

#line 20
  return __nesc_result;
#line 20
}
#line 20
# 30 "/Users/jingyuancheng/tinyos/moteiv/tos/platform/msp430/timer/MSP430Timer.nc"
inline static uint16_t /*CC2420RadioC.BackoffAlarm32khzC.MSP430Alarm*/MSP430AlarmC$1$MSP430Timer$get(void ){
#line 30
  unsigned int __nesc_result;
#line 30

#line 30
  __nesc_result = /*MSP430TimerC.MSP430TimerB*/MSP430TimerM$1$Timer$get();
#line 30

#line 30
  return __nesc_result;
#line 30
}
#line 30
# 95 "/Users/jingyuancheng/tinyos/moteiv/tos/platform/msp430/timer/MSP430AlarmC.nc"
static inline uint16_t /*CC2420RadioC.BackoffAlarm32khzC.MSP430Alarm*/MSP430AlarmC$1$Alarm$getNow(void )
{
  return /*CC2420RadioC.BackoffAlarm32khzC.MSP430Alarm*/MSP430AlarmC$1$MSP430Timer$get();
}

#line 51
static inline void /*CC2420RadioC.BackoffAlarm32khzC.MSP430Alarm*/MSP430AlarmC$1$Alarm$start(uint16_t dt)
{
  /*CC2420RadioC.BackoffAlarm32khzC.MSP430Alarm*/MSP430AlarmC$1$Alarm$startAt(/*CC2420RadioC.BackoffAlarm32khzC.MSP430Alarm*/MSP430AlarmC$1$Alarm$getNow(), dt);
}

# 54 "/Users/jingyuancheng/tinyos/moteiv/tos/lib/timer/Alarm.nc"
inline static void CC2420RadioM$BackoffAlarm32khz$start(CC2420RadioM$BackoffAlarm32khz$size_type dt){
#line 54
  /*CC2420RadioC.BackoffAlarm32khzC.MSP430Alarm*/MSP430AlarmC$1$Alarm$start(dt);
#line 54
}
#line 54
# 123 "/Users/jingyuancheng/tinyos/moteiv/tos/platform/msp430/timer/MSP430TimerCapComM.nc"
static inline void /*MSP430TimerC.MSP430TimerB3*/MSP430TimerCapComM$6$Control$disableEvents(void )
{
  * (volatile uint16_t *)392U &= ~0x0010;
}

# 39 "/Users/jingyuancheng/tinyos/moteiv/tos/platform/msp430/timer/MSP430TimerControl.nc"
inline static void /*CC2420RadioC.BackoffAlarm32khzC.MSP430Alarm*/MSP430AlarmC$1$MSP430TimerControl$disableEvents(void ){
#line 39
  /*MSP430TimerC.MSP430TimerB3*/MSP430TimerCapComM$6$Control$disableEvents();
#line 39
}
#line 39
# 56 "/Users/jingyuancheng/tinyos/moteiv/tos/platform/msp430/timer/MSP430AlarmC.nc"
static inline void /*CC2420RadioC.BackoffAlarm32khzC.MSP430Alarm*/MSP430AlarmC$1$Alarm$stop(void )
{
  /*CC2420RadioC.BackoffAlarm32khzC.MSP430Alarm*/MSP430AlarmC$1$MSP430TimerControl$disableEvents();
}

# 60 "/Users/jingyuancheng/tinyos/moteiv/tos/lib/timer/Alarm.nc"
inline static void CC2420RadioM$BackoffAlarm32khz$stop(void ){
#line 60
  /*CC2420RadioC.BackoffAlarm32khzC.MSP430Alarm*/MSP430AlarmC$1$Alarm$stop();
#line 60
}
#line 60
# 128 "/Users/jingyuancheng/tinyos/moteiv/tos/platform/msp430/timer/MSP430TimerCapComM.nc"
static inline bool /*MSP430TimerC.MSP430TimerB3*/MSP430TimerCapComM$6$Control$areEventsEnabled(void )
{
  return * (volatile uint16_t *)392U & 0x0010;
}

# 40 "/Users/jingyuancheng/tinyos/moteiv/tos/platform/msp430/timer/MSP430TimerControl.nc"
inline static bool /*CC2420RadioC.BackoffAlarm32khzC.MSP430Alarm*/MSP430AlarmC$1$MSP430TimerControl$areEventsEnabled(void ){
#line 40
  unsigned char __nesc_result;
#line 40

#line 40
  __nesc_result = /*MSP430TimerC.MSP430TimerB3*/MSP430TimerCapComM$6$Control$areEventsEnabled();
#line 40

#line 40
  return __nesc_result;
#line 40
}
#line 40
# 67 "/Users/jingyuancheng/tinyos/moteiv/tos/platform/msp430/timer/MSP430AlarmC.nc"
static inline bool /*CC2420RadioC.BackoffAlarm32khzC.MSP430Alarm*/MSP430AlarmC$1$Alarm$isRunning(void )
{
  return /*CC2420RadioC.BackoffAlarm32khzC.MSP430Alarm*/MSP430AlarmC$1$MSP430TimerControl$areEventsEnabled();
}

# 74 "/Users/jingyuancheng/tinyos/moteiv/tos/lib/timer/Alarm.nc"
inline static bool CC2420RadioM$BackoffAlarm32khz$isRunning(void ){
#line 74
  unsigned char __nesc_result;
#line 74

#line 74
  __nesc_result = /*CC2420RadioC.BackoffAlarm32khzC.MSP430Alarm*/MSP430AlarmC$1$Alarm$isRunning();
#line 74

#line 74
  return __nesc_result;
#line 74
}
#line 74
# 43 "/Users/jingyuancheng/tinyos/moteiv/tinyos-1.x/tos/lib/CC2420Radio/HPLCC2420Capture.nc"
inline static result_t CC2420RadioM$SFD$enableCapture(bool low_to_high){
#line 43
  unsigned char __nesc_result;
#line 43

#line 43
  __nesc_result = HPLCC2420InterruptM$SFD$enableCapture(low_to_high);
#line 43

#line 43
  return __nesc_result;
#line 43
}
#line 43
# 52 "/Users/jingyuancheng/tinyos/moteiv/tos/lib/timer/Counter.nc"
inline static CC2420RadioM$Counter32khz$size_type CC2420RadioM$Counter32khz$get(void ){
#line 52
  unsigned long __nesc_result;
#line 52

#line 52
  __nesc_result = /*Counter32khzC.Transform*/TransformCounterC$1$Counter$get();
#line 52

#line 52
  return __nesc_result;
#line 52
}
#line 52
# 155 "/Users/jingyuancheng/tinyos/moteiv/tos/lib/resource/FcfsArbiterP.nc"
static inline void /*MSP430ArbiterUSART0C.ArbiterC.ArbiterP*/FcfsArbiterP$1$ResourceCmd$deferRequest(uint8_t id)
#line 155
{
  /*MSP430ArbiterUSART0C.ArbiterC.ArbiterP*/FcfsArbiterP$1$Resource$request(id);
}

# 63 "/Users/jingyuancheng/tinyos/moteiv/tos/lib/resource/ResourceCmd.nc"
inline static void CC2420RadioM$CmdReceive$deferRequest(void ){
#line 63
  /*MSP430ArbiterUSART0C.ArbiterC.ArbiterP*/FcfsArbiterP$1$ResourceCmd$deferRequest(/*CC2420RadioC.CmdReceiveC.ResourceC.ResourceUSARTP*/MSP430ResourceUSART0P$6$ID);
#line 63
}
#line 63
# 159 "/Users/jingyuancheng/tinyos/moteiv/tos/lib/CC2420Radio/CC2420RadioM.nc"
static inline void CC2420RadioM$sendFailedAsync(void )
#line 159
{
  CC2420RadioM$sendFailedTask$postTask();
}

#line 204
static __inline result_t CC2420RadioM$setAckTimer(uint16_t jiffy)
#line 204
{
  CC2420RadioM$stateTimer = CC2420RadioM$TIMER_ACK;
  CC2420RadioM$BackoffAlarm32khz$start(jiffy);
  return SUCCESS;
}

# 41 "/Users/jingyuancheng/tinyos/moteiv/tos/platform/tmote/hardware.h"
static inline void TOSH_CLR_UTXD0_PIN()
#line 41
{
#line 41
  static volatile uint8_t r __asm ("0x0019");

#line 41
  r &= ~(1 << 4);
}

# 60 "/Users/jingyuancheng/tinyos/moteiv/tinyos-1.x/tos/lib/CC2420Radio/HPLCC2420Capture.nc"
inline static result_t CC2420RadioM$SFD$disable(void ){
#line 60
  unsigned char __nesc_result;
#line 60

#line 60
  __nesc_result = HPLCC2420InterruptM$SFD$disable();
#line 60

#line 60
  return __nesc_result;
#line 60
}
#line 60
# 32 "/Users/jingyuancheng/tinyos/moteiv/tos/platform/tmote/hardware.h"
static inline uint8_t TOSH_READ_CC_SFD_PIN()
#line 32
{
#line 32
  static volatile uint8_t r __asm ("0x001C");

#line 32
  return r & (1 << 1);
}

# 65 "/Users/jingyuancheng/tinyos/moteiv/tos/lib/resource/ResourceCmdAsync.nc"
inline static void CC2420TimeStampingM$CmdWriteTimeStamp$urgentRequest(uint8_t rh){
#line 65
  /*MSP430ArbiterUSART0C.ArbiterC.ArbiterP*/FcfsArbiterP$1$ResourceCmdAsync$urgentRequest(/*CC2420TimeStampingC.CmdWriteTimeStampC.ResourceC.ResourceUSARTP*/MSP430ResourceUSART0P$9$ID, rh);
#line 65
}
#line 65
# 52 "/Users/jingyuancheng/tinyos/moteiv/tos/lib/timer/Counter.nc"
inline static /*Counter32khzC.CounterToLocalTimeC*/CounterToLocalTimeC$1$Counter$size_type /*Counter32khzC.CounterToLocalTimeC*/CounterToLocalTimeC$1$Counter$get(void ){
#line 52
  unsigned long __nesc_result;
#line 52

#line 52
  __nesc_result = /*Counter32khzC.Transform*/TransformCounterC$1$Counter$get();
#line 52

#line 52
  return __nesc_result;
#line 52
}
#line 52
# 36 "/Users/jingyuancheng/tinyos/moteiv/tos/lib/timer/CounterToLocalTimeC.nc"
static inline uint32_t /*Counter32khzC.CounterToLocalTimeC*/CounterToLocalTimeC$1$LocalTime$get(void )
{
  return /*Counter32khzC.CounterToLocalTimeC*/CounterToLocalTimeC$1$Counter$get();
}

# 49 "/Users/jingyuancheng/tinyos/moteiv/tos/lib/timer/LocalTime.nc"
inline static uint32_t CC2420TimeStampingM$LocalTime$get(void ){
#line 49
  unsigned long __nesc_result;
#line 49

#line 49
  __nesc_result = /*Counter32khzC.CounterToLocalTimeC*/CounterToLocalTimeC$1$LocalTime$get();
#line 49

#line 49
  return __nesc_result;
#line 49
}
#line 49
# 52 "/Users/jingyuancheng/tinyos/moteiv/tos/lib/sp/cc2420/CC2420TimeStampingM.nc"
static inline void CC2420TimeStampingM$RadioSendCoordinator$startSymbol(uint8_t bitsPerBlock, uint8_t offset, TOS_MsgPtr msgBuff)
{
  uint32_t send_time;

  /* atomic removed: atomic calls only */
#line 56
  send_time = CC2420TimeStampingM$LocalTime$get() - CC2420TimeStampingM$SEND_TIME_CORRECTION;

  if (CC2420TimeStampingM$ptosMsg != NULL && CC2420TimeStampingM$ptosMsg != msgBuff) {
    return;
    }
  if (CC2420TimeStampingM$sendStampOffset < 0) {
    return;
    }
  * (uint32_t *)((void *)msgBuff->data + CC2420TimeStampingM$sendStampOffset) = send_time;

  CC2420TimeStampingM$timestampMsgBuf = msgBuff;

  CC2420TimeStampingM$CmdWriteTimeStamp$urgentRequest(RESOURCE_NONE);
}

# 33 "/Users/jingyuancheng/tinyos/moteiv/tinyos-1.x/tos/interfaces/RadioCoordinator.nc"
inline static void CC2420RadioM$RadioSendCoordinator$startSymbol(uint8_t bitsPerBlock, uint8_t offset, TOS_MsgPtr msgBuff){
#line 33
  CC2420TimeStampingM$RadioSendCoordinator$startSymbol(bitsPerBlock, offset, msgBuff);
#line 33
}
#line 33
# 458 "/Users/jingyuancheng/tinyos/moteiv/tos/lib/CC2420Radio/CC2420RadioM.nc"
static inline result_t CC2420RadioM$SFD$captured(uint16_t time)
#line 458
{
  switch (CC2420RadioM$stateRadio) {
      case CC2420RadioM$TX_STATE: 

        CC2420RadioM$SFD$enableCapture(FALSE);

      CC2420RadioM$txbufptr->time = time;
      CC2420RadioM$RadioSendCoordinator$startSymbol(8, 0, CC2420RadioM$txbufptr);




      if (!TOSH_READ_CC_SFD_PIN()) {
          CC2420RadioM$SFD$disable();
        }
      else {


          CC2420RadioM$stateRadio = CC2420RadioM$TX_WAIT;
          break;
        }
      case CC2420RadioM$TX_WAIT: 

        CC2420RadioM$stateRadio = CC2420RadioM$POST_TX_STATE;
      CC2420RadioM$SFD$disable();

      CC2420RadioM$SFD$enableCapture(TRUE);



      CC2420RadioM$BackoffAlarm32khz$stop();

      TOSH_CLR_UTXD0_PIN();


      if (CC2420RadioM$txbufptr->fcfhi == 0x21 && 
      CC2420RadioM$txbufptr->addr != TOS_BCAST_ADDR) {
          if (!CC2420RadioM$setAckTimer(200)) {
            CC2420RadioM$sendFailedAsync();
            }
        }
      else {
          if (!CC2420RadioM$PacketSent$postTask()) {
            CC2420RadioM$sendFailedAsync();
            }
        }
#line 503
      break;
      default: 
        if (CC2420RadioM$m_sfdReceiving) {
            CC2420RadioM$m_sfdReceiving = FALSE;
            CC2420RadioM$SFD$enableCapture(TRUE);
            if (TOSH_READ_CC_FIFO_PIN()) {

                CC2420RadioM$m_rxFifoCount++;
                CC2420RadioM$CmdReceive$deferRequest();
              }
            else 
#line 512
              {

                CC2420RadioM$flushRXFIFO(RESOURCE_NONE);
              }
          }
        else 
#line 516
          {
            uint32_t when = CC2420RadioM$Counter32khz$get();

#line 518
            if ((when & 0xFFFF) < time) {
              when -= 0x10000L;
              }
#line 520
            when = (when & 0xffff0000L) | time;


            CC2420RadioM$m_sfdReceiving = TRUE;
            CC2420RadioM$SFD$enableCapture(FALSE);




            if (CC2420RadioM$stateRadio == CC2420RadioM$PRE_TX_STATE && CC2420RadioM$BackoffAlarm32khz$isRunning()) {
                CC2420RadioM$BackoffAlarm32khz$stop();
                CC2420RadioM$BackoffAlarm32khz$start(CC2420RadioM$MacBackoff$congestionBackoff(CC2420RadioM$txbufptr) * 2 + 75);
              }

            if (cqueue_pushBack(&CC2420RadioM$m_timestampQueue)) {
              CC2420RadioM$m_timestamps[CC2420RadioM$m_timestampQueue.back] = when;
              }
            CC2420RadioM$RadioReceiveCoordinator$startSymbol(8, 0, CC2420RadioM$rxbufptr);
          }
    }
  return SUCCESS;
}

# 53 "/Users/jingyuancheng/tinyos/moteiv/tinyos-1.x/tos/lib/CC2420Radio/HPLCC2420Capture.nc"
inline static result_t HPLCC2420InterruptM$SFD$captured(uint16_t val){
#line 53
  unsigned char __nesc_result;
#line 53

#line 53
  __nesc_result = CC2420RadioM$SFD$captured(val);
#line 53

#line 53
  return __nesc_result;
#line 53
}
#line 53
# 217 "/Users/jingyuancheng/tinyos/moteiv/tos/platform/tmote/HPLCC2420InterruptM.nc"
static inline void HPLCC2420InterruptM$SFDCapture$captured(uint16_t time)
#line 217
{
  result_t val = SUCCESS;

#line 219
  HPLCC2420InterruptM$SFDControl$clearPendingInterrupt();
  val = HPLCC2420InterruptM$SFD$captured(time);
  if (val == FAIL) {
      HPLCC2420InterruptM$SFDControl$disableEvents();
      HPLCC2420InterruptM$SFDControl$clearPendingInterrupt();
    }
  else {
      if (HPLCC2420InterruptM$SFDCapture$isOverflowPending()) {
        HPLCC2420InterruptM$SFDCapture$clearOverflow();
        }
    }
}

# 74 "/Users/jingyuancheng/tinyos/moteiv/tos/platform/msp430/timer/MSP430Capture.nc"
inline static void /*MSP430TimerC.MSP430TimerB1*/MSP430TimerCapComM$4$Capture$captured(uint16_t time){
#line 74
  HPLCC2420InterruptM$SFDCapture$captured(time);
#line 74
}
#line 74
# 46 "/Users/jingyuancheng/tinyos/moteiv/tos/platform/msp430/timer/MSP430TimerCapComM.nc"
static inline /*MSP430TimerC.MSP430TimerB1*/MSP430TimerCapComM$4$CC_t /*MSP430TimerC.MSP430TimerB1*/MSP430TimerCapComM$4$int2CC(uint16_t x)
#line 46
{
#line 46
  union /*MSP430TimerC.MSP430TimerB1*/MSP430TimerCapComM$4$__nesc_unnamed4390 {
#line 46
    uint16_t f;
#line 46
    /*MSP430TimerC.MSP430TimerB1*/MSP430TimerCapComM$4$CC_t t;
  } 
#line 46
  c = { .f = x };

#line 46
  return c.t;
}

#line 73
static inline /*MSP430TimerC.MSP430TimerB1*/MSP430TimerCapComM$4$CC_t /*MSP430TimerC.MSP430TimerB1*/MSP430TimerCapComM$4$Control$getControl(void )
{
  return /*MSP430TimerC.MSP430TimerB1*/MSP430TimerCapComM$4$int2CC(* (volatile uint16_t *)388U);
}

#line 168
static inline void /*MSP430TimerC.MSP430TimerB1*/MSP430TimerCapComM$4$Event$fired(void )
{
  if (/*MSP430TimerC.MSP430TimerB1*/MSP430TimerCapComM$4$Control$getControl().cap) {
    /*MSP430TimerC.MSP430TimerB1*/MSP430TimerCapComM$4$Capture$captured(/*MSP430TimerC.MSP430TimerB1*/MSP430TimerCapComM$4$Capture$getEvent());
    }
  else {
#line 173
    /*MSP430TimerC.MSP430TimerB1*/MSP430TimerCapComM$4$Compare$fired();
    }
}

# 30 "/Users/jingyuancheng/tinyos/moteiv/tos/platform/msp430/timer/MSP430Timer.nc"
inline static uint16_t /*MSP430Counter32khzC.Counter*/MSP430CounterC$0$MSP430Timer$get(void ){
#line 30
  unsigned int __nesc_result;
#line 30

#line 30
  __nesc_result = /*MSP430TimerC.MSP430TimerB*/MSP430TimerM$1$Timer$get();
#line 30

#line 30
  return __nesc_result;
#line 30
}
#line 30
# 36 "/Users/jingyuancheng/tinyos/moteiv/tos/platform/msp430/timer/MSP430CounterC.nc"
static inline uint16_t /*MSP430Counter32khzC.Counter*/MSP430CounterC$0$Counter$get(void )
{
  return /*MSP430Counter32khzC.Counter*/MSP430CounterC$0$MSP430Timer$get();
}

# 52 "/Users/jingyuancheng/tinyos/moteiv/tos/lib/timer/Counter.nc"
inline static /*Counter32khzC.Transform*/TransformCounterC$1$CounterFrom$size_type /*Counter32khzC.Transform*/TransformCounterC$1$CounterFrom$get(void ){
#line 52
  unsigned int __nesc_result;
#line 52

#line 52
  __nesc_result = /*MSP430Counter32khzC.Counter*/MSP430CounterC$0$Counter$get();
#line 52

#line 52
  return __nesc_result;
#line 52
}
#line 52
# 69 "/Users/jingyuancheng/tinyos/moteiv/tos/platform/msp430/timer/MSP430TimerM.nc"
static inline bool /*MSP430TimerC.MSP430TimerB*/MSP430TimerM$1$Timer$isOverflowPending(void )
{
  return * (volatile uint16_t *)384U & 1U;
}

# 31 "/Users/jingyuancheng/tinyos/moteiv/tos/platform/msp430/timer/MSP430Timer.nc"
inline static bool /*MSP430Counter32khzC.Counter*/MSP430CounterC$0$MSP430Timer$isOverflowPending(void ){
#line 31
  unsigned char __nesc_result;
#line 31

#line 31
  __nesc_result = /*MSP430TimerC.MSP430TimerB*/MSP430TimerM$1$Timer$isOverflowPending();
#line 31

#line 31
  return __nesc_result;
#line 31
}
#line 31
# 41 "/Users/jingyuancheng/tinyos/moteiv/tos/platform/msp430/timer/MSP430CounterC.nc"
static inline bool /*MSP430Counter32khzC.Counter*/MSP430CounterC$0$Counter$isOverflowPending(void )
{
  return /*MSP430Counter32khzC.Counter*/MSP430CounterC$0$MSP430Timer$isOverflowPending();
}

# 59 "/Users/jingyuancheng/tinyos/moteiv/tos/lib/timer/Counter.nc"
inline static bool /*Counter32khzC.Transform*/TransformCounterC$1$CounterFrom$isOverflowPending(void ){
#line 59
  unsigned char __nesc_result;
#line 59

#line 59
  __nesc_result = /*MSP430Counter32khzC.Counter*/MSP430CounterC$0$Counter$isOverflowPending();
#line 59

#line 59
  return __nesc_result;
#line 59
}
#line 59
# 32 "/Users/jingyuancheng/tinyos/moteiv/tos/platform/tmote/hardware.h"
static inline void TOSH_SEL_CC_SFD_IOFUNC()
#line 32
{
#line 32
  static volatile uint8_t r __asm ("0x001F");

#line 32
  r &= ~(1 << 1);
}

# 30 "/Users/jingyuancheng/tinyos/moteiv/tos/platform/msp430/timer/MSP430Timer.nc"
inline static uint16_t /*MSP430TimerC.MSP430TimerB3*/MSP430TimerCapComM$6$Timer$get(void ){
#line 30
  unsigned int __nesc_result;
#line 30

#line 30
  __nesc_result = /*MSP430TimerC.MSP430TimerB*/MSP430TimerM$1$Timer$get();
#line 30

#line 30
  return __nesc_result;
#line 30
}
#line 30
# 153 "/Users/jingyuancheng/tinyos/moteiv/tos/platform/msp430/timer/MSP430TimerCapComM.nc"
static inline void /*MSP430TimerC.MSP430TimerB3*/MSP430TimerCapComM$6$Compare$setEventFromNow(uint16_t x)
{
  * (volatile uint16_t *)408U = /*MSP430TimerC.MSP430TimerB3*/MSP430TimerCapComM$6$Timer$get() + x;
}

# 32 "/Users/jingyuancheng/tinyos/moteiv/tos/platform/msp430/timer/MSP430Compare.nc"
inline static void /*CC2420RadioC.BackoffAlarm32khzC.MSP430Alarm*/MSP430AlarmC$1$MSP430Compare$setEventFromNow(uint16_t delta){
#line 32
  /*MSP430TimerC.MSP430TimerB3*/MSP430TimerCapComM$6$Compare$setEventFromNow(delta);
#line 32
}
#line 32
# 143 "/Users/jingyuancheng/tinyos/moteiv/tos/platform/msp430/timer/MSP430TimerCapComM.nc"
static inline void /*MSP430TimerC.MSP430TimerB3*/MSP430TimerCapComM$6$Compare$setEvent(uint16_t x)
{
  * (volatile uint16_t *)408U = x;
}

# 30 "/Users/jingyuancheng/tinyos/moteiv/tos/platform/msp430/timer/MSP430Compare.nc"
inline static void /*CC2420RadioC.BackoffAlarm32khzC.MSP430Alarm*/MSP430AlarmC$1$MSP430Compare$setEvent(uint16_t time){
#line 30
  /*MSP430TimerC.MSP430TimerB3*/MSP430TimerCapComM$6$Compare$setEvent(time);
#line 30
}
#line 30
# 83 "/Users/jingyuancheng/tinyos/moteiv/tos/platform/msp430/timer/MSP430TimerCapComM.nc"
static inline void /*MSP430TimerC.MSP430TimerB3*/MSP430TimerCapComM$6$Control$clearPendingInterrupt(void )
{
  * (volatile uint16_t *)392U &= ~0x0001;
}

# 32 "/Users/jingyuancheng/tinyos/moteiv/tos/platform/msp430/timer/MSP430TimerControl.nc"
inline static void /*CC2420RadioC.BackoffAlarm32khzC.MSP430Alarm*/MSP430AlarmC$1$MSP430TimerControl$clearPendingInterrupt(void ){
#line 32
  /*MSP430TimerC.MSP430TimerB3*/MSP430TimerCapComM$6$Control$clearPendingInterrupt();
#line 32
}
#line 32
# 118 "/Users/jingyuancheng/tinyos/moteiv/tos/platform/msp430/timer/MSP430TimerCapComM.nc"
static inline void /*MSP430TimerC.MSP430TimerB3*/MSP430TimerCapComM$6$Control$enableEvents(void )
{
  * (volatile uint16_t *)392U |= 0x0010;
}

# 38 "/Users/jingyuancheng/tinyos/moteiv/tos/platform/msp430/timer/MSP430TimerControl.nc"
inline static void /*CC2420RadioC.BackoffAlarm32khzC.MSP430Alarm*/MSP430AlarmC$1$MSP430TimerControl$enableEvents(void ){
#line 38
  /*MSP430TimerC.MSP430TimerB3*/MSP430TimerCapComM$6$Control$enableEvents();
#line 38
}
#line 38
# 49 "/Users/jingyuancheng/tinyos/moteiv/tos/lib/sched/TaskBasic.nc"
inline static result_t /*HalTimerMilliC.AlarmToTimerC*/AlarmToTimerC$0$fired$postTask(void ){
#line 49
  unsigned char __nesc_result;
#line 49

#line 49
  __nesc_result = SchedulerBasicP$TaskBasic$postTask(/*HalTimerMilliC.AlarmToTimerC*/AlarmToTimerC$0$fired);
#line 49

#line 49
  return __nesc_result;
#line 49
}
#line 49
# 63 "/Users/jingyuancheng/tinyos/moteiv/tos/lib/timer/AlarmToTimerC.nc"
static inline void /*HalTimerMilliC.AlarmToTimerC*/AlarmToTimerC$0$Alarm$fired(void )
#line 63
{
  /*HalTimerMilliC.AlarmToTimerC*/AlarmToTimerC$0$fired$postTask();
}

# 64 "/Users/jingyuancheng/tinyos/moteiv/tos/lib/timer/Alarm.nc"
inline static void /*HalTimerMilliC.AlarmMilliC.Transform*/TransformAlarmC$0$Alarm$fired(void ){
#line 64
  /*HalTimerMilliC.AlarmToTimerC*/AlarmToTimerC$0$Alarm$fired();
#line 64
}
#line 64
# 115 "/Users/jingyuancheng/tinyos/moteiv/tos/lib/timer/TransformAlarmC.nc"
static inline void /*HalTimerMilliC.AlarmMilliC.Transform*/TransformAlarmC$0$AlarmFrom$fired(void )
{
  /* atomic removed: atomic calls only */
  {
    if (/*HalTimerMilliC.AlarmMilliC.Transform*/TransformAlarmC$0$m_dt == 0) 
      {
        /*HalTimerMilliC.AlarmMilliC.Transform*/TransformAlarmC$0$Alarm$fired();
      }
    else 
      {
        /*HalTimerMilliC.AlarmMilliC.Transform*/TransformAlarmC$0$set_alarm();
      }
  }
}

# 64 "/Users/jingyuancheng/tinyos/moteiv/tos/lib/timer/Alarm.nc"
inline static void /*HalTimerMilliC.AlarmMilliC.MSP430Alarm*/MSP430AlarmC$0$Alarm$fired(void ){
#line 64
  /*HalTimerMilliC.AlarmMilliC.Transform*/TransformAlarmC$0$AlarmFrom$fired();
#line 64
}
#line 64
# 123 "/Users/jingyuancheng/tinyos/moteiv/tos/platform/msp430/timer/MSP430TimerCapComM.nc"
static inline void /*MSP430TimerC.MSP430TimerB2*/MSP430TimerCapComM$5$Control$disableEvents(void )
{
  * (volatile uint16_t *)390U &= ~0x0010;
}

# 39 "/Users/jingyuancheng/tinyos/moteiv/tos/platform/msp430/timer/MSP430TimerControl.nc"
inline static void /*HalTimerMilliC.AlarmMilliC.MSP430Alarm*/MSP430AlarmC$0$MSP430TimerControl$disableEvents(void ){
#line 39
  /*MSP430TimerC.MSP430TimerB2*/MSP430TimerCapComM$5$Control$disableEvents();
#line 39
}
#line 39
# 61 "/Users/jingyuancheng/tinyos/moteiv/tos/platform/msp430/timer/MSP430AlarmC.nc"
static inline void /*HalTimerMilliC.AlarmMilliC.MSP430Alarm*/MSP430AlarmC$0$MSP430Compare$fired(void )
{
  /*HalTimerMilliC.AlarmMilliC.MSP430Alarm*/MSP430AlarmC$0$MSP430TimerControl$disableEvents();
  /*HalTimerMilliC.AlarmMilliC.MSP430Alarm*/MSP430AlarmC$0$Alarm$fired();
}

# 34 "/Users/jingyuancheng/tinyos/moteiv/tos/platform/msp430/timer/MSP430Compare.nc"
inline static void /*MSP430TimerC.MSP430TimerB2*/MSP430TimerCapComM$5$Compare$fired(void ){
#line 34
  /*HalTimerMilliC.AlarmMilliC.MSP430Alarm*/MSP430AlarmC$0$MSP430Compare$fired();
#line 34
}
#line 34
# 138 "/Users/jingyuancheng/tinyos/moteiv/tos/platform/msp430/timer/MSP430TimerCapComM.nc"
static inline uint16_t /*MSP430TimerC.MSP430TimerB2*/MSP430TimerCapComM$5$Capture$getEvent(void )
{
  return * (volatile uint16_t *)406U;
}

#line 176
static inline void /*MSP430TimerC.MSP430TimerB2*/MSP430TimerCapComM$5$Capture$default$captured(uint16_t n)
{
}

# 74 "/Users/jingyuancheng/tinyos/moteiv/tos/platform/msp430/timer/MSP430Capture.nc"
inline static void /*MSP430TimerC.MSP430TimerB2*/MSP430TimerCapComM$5$Capture$captured(uint16_t time){
#line 74
  /*MSP430TimerC.MSP430TimerB2*/MSP430TimerCapComM$5$Capture$default$captured(time);
#line 74
}
#line 74
# 46 "/Users/jingyuancheng/tinyos/moteiv/tos/platform/msp430/timer/MSP430TimerCapComM.nc"
static inline /*MSP430TimerC.MSP430TimerB2*/MSP430TimerCapComM$5$CC_t /*MSP430TimerC.MSP430TimerB2*/MSP430TimerCapComM$5$int2CC(uint16_t x)
#line 46
{
#line 46
  union /*MSP430TimerC.MSP430TimerB2*/MSP430TimerCapComM$5$__nesc_unnamed4391 {
#line 46
    uint16_t f;
#line 46
    /*MSP430TimerC.MSP430TimerB2*/MSP430TimerCapComM$5$CC_t t;
  } 
#line 46
  c = { .f = x };

#line 46
  return c.t;
}

#line 73
static inline /*MSP430TimerC.MSP430TimerB2*/MSP430TimerCapComM$5$CC_t /*MSP430TimerC.MSP430TimerB2*/MSP430TimerCapComM$5$Control$getControl(void )
{
  return /*MSP430TimerC.MSP430TimerB2*/MSP430TimerCapComM$5$int2CC(* (volatile uint16_t *)390U);
}

#line 168
static inline void /*MSP430TimerC.MSP430TimerB2*/MSP430TimerCapComM$5$Event$fired(void )
{
  if (/*MSP430TimerC.MSP430TimerB2*/MSP430TimerCapComM$5$Control$getControl().cap) {
    /*MSP430TimerC.MSP430TimerB2*/MSP430TimerCapComM$5$Capture$captured(/*MSP430TimerC.MSP430TimerB2*/MSP430TimerCapComM$5$Capture$getEvent());
    }
  else {
#line 173
    /*MSP430TimerC.MSP430TimerB2*/MSP430TimerCapComM$5$Compare$fired();
    }
}

# 52 "/Users/jingyuancheng/tinyos/moteiv/tos/lib/timer/Counter.nc"
inline static /*CounterMilliC.Transform*/TransformCounterC$0$CounterFrom$size_type /*CounterMilliC.Transform*/TransformCounterC$0$CounterFrom$get(void ){
#line 52
  unsigned int __nesc_result;
#line 52

#line 52
  __nesc_result = /*MSP430Counter32khzC.Counter*/MSP430CounterC$0$Counter$get();
#line 52

#line 52
  return __nesc_result;
#line 52
}
#line 52







inline static bool /*CounterMilliC.Transform*/TransformCounterC$0$CounterFrom$isOverflowPending(void ){
#line 59
  unsigned char __nesc_result;
#line 59

#line 59
  __nesc_result = /*MSP430Counter32khzC.Counter*/MSP430CounterC$0$Counter$isOverflowPending();
#line 59

#line 59
  return __nesc_result;
#line 59
}
#line 59
# 88 "/Users/jingyuancheng/tinyos/moteiv/tos/lib/timer/Alarm.nc"
inline static void /*HalTimerMilliC.AlarmMilliC.Transform*/TransformAlarmC$0$AlarmFrom$startAt(/*HalTimerMilliC.AlarmMilliC.Transform*/TransformAlarmC$0$AlarmFrom$size_type t0, /*HalTimerMilliC.AlarmMilliC.Transform*/TransformAlarmC$0$AlarmFrom$size_type dt){
#line 88
  /*HalTimerMilliC.AlarmMilliC.MSP430Alarm*/MSP430AlarmC$0$Alarm$startAt(t0, dt);
#line 88
}
#line 88
# 30 "/Users/jingyuancheng/tinyos/moteiv/tos/platform/msp430/timer/MSP430Timer.nc"
inline static uint16_t /*HalTimerMilliC.AlarmMilliC.MSP430Alarm*/MSP430AlarmC$0$MSP430Timer$get(void ){
#line 30
  unsigned int __nesc_result;
#line 30

#line 30
  __nesc_result = /*MSP430TimerC.MSP430TimerB*/MSP430TimerM$1$Timer$get();
#line 30

#line 30
  return __nesc_result;
#line 30
}
#line 30
inline static uint16_t /*MSP430TimerC.MSP430TimerB2*/MSP430TimerCapComM$5$Timer$get(void ){
#line 30
  unsigned int __nesc_result;
#line 30

#line 30
  __nesc_result = /*MSP430TimerC.MSP430TimerB*/MSP430TimerM$1$Timer$get();
#line 30

#line 30
  return __nesc_result;
#line 30
}
#line 30
# 153 "/Users/jingyuancheng/tinyos/moteiv/tos/platform/msp430/timer/MSP430TimerCapComM.nc"
static inline void /*MSP430TimerC.MSP430TimerB2*/MSP430TimerCapComM$5$Compare$setEventFromNow(uint16_t x)
{
  * (volatile uint16_t *)406U = /*MSP430TimerC.MSP430TimerB2*/MSP430TimerCapComM$5$Timer$get() + x;
}

# 32 "/Users/jingyuancheng/tinyos/moteiv/tos/platform/msp430/timer/MSP430Compare.nc"
inline static void /*HalTimerMilliC.AlarmMilliC.MSP430Alarm*/MSP430AlarmC$0$MSP430Compare$setEventFromNow(uint16_t delta){
#line 32
  /*MSP430TimerC.MSP430TimerB2*/MSP430TimerCapComM$5$Compare$setEventFromNow(delta);
#line 32
}
#line 32
# 143 "/Users/jingyuancheng/tinyos/moteiv/tos/platform/msp430/timer/MSP430TimerCapComM.nc"
static inline void /*MSP430TimerC.MSP430TimerB2*/MSP430TimerCapComM$5$Compare$setEvent(uint16_t x)
{
  * (volatile uint16_t *)406U = x;
}

# 30 "/Users/jingyuancheng/tinyos/moteiv/tos/platform/msp430/timer/MSP430Compare.nc"
inline static void /*HalTimerMilliC.AlarmMilliC.MSP430Alarm*/MSP430AlarmC$0$MSP430Compare$setEvent(uint16_t time){
#line 30
  /*MSP430TimerC.MSP430TimerB2*/MSP430TimerCapComM$5$Compare$setEvent(time);
#line 30
}
#line 30
# 83 "/Users/jingyuancheng/tinyos/moteiv/tos/platform/msp430/timer/MSP430TimerCapComM.nc"
static inline void /*MSP430TimerC.MSP430TimerB2*/MSP430TimerCapComM$5$Control$clearPendingInterrupt(void )
{
  * (volatile uint16_t *)390U &= ~0x0001;
}

# 32 "/Users/jingyuancheng/tinyos/moteiv/tos/platform/msp430/timer/MSP430TimerControl.nc"
inline static void /*HalTimerMilliC.AlarmMilliC.MSP430Alarm*/MSP430AlarmC$0$MSP430TimerControl$clearPendingInterrupt(void ){
#line 32
  /*MSP430TimerC.MSP430TimerB2*/MSP430TimerCapComM$5$Control$clearPendingInterrupt();
#line 32
}
#line 32
# 118 "/Users/jingyuancheng/tinyos/moteiv/tos/platform/msp430/timer/MSP430TimerCapComM.nc"
static inline void /*MSP430TimerC.MSP430TimerB2*/MSP430TimerCapComM$5$Control$enableEvents(void )
{
  * (volatile uint16_t *)390U |= 0x0010;
}

# 38 "/Users/jingyuancheng/tinyos/moteiv/tos/platform/msp430/timer/MSP430TimerControl.nc"
inline static void /*HalTimerMilliC.AlarmMilliC.MSP430Alarm*/MSP430AlarmC$0$MSP430TimerControl$enableEvents(void ){
#line 38
  /*MSP430TimerC.MSP430TimerB2*/MSP430TimerCapComM$5$Control$enableEvents();
#line 38
}
#line 38
# 63 "/Users/jingyuancheng/tinyos/moteiv/tos/lib/resource/ResourceCmd.nc"
inline static void CC2420RadioM$CmdTryToSend$deferRequest(void ){
#line 63
  /*MSP430ArbiterUSART0C.ArbiterC.ArbiterP*/FcfsArbiterP$1$ResourceCmd$deferRequest(/*CC2420RadioC.CmdTryToSendC.ResourceC.ResourceUSARTP*/MSP430ResourceUSART0P$8$ID);
#line 63
}
#line 63
inline static void CC2420RadioM$CmdTransmit$deferRequest(void ){
#line 63
  /*MSP430ArbiterUSART0C.ArbiterC.ArbiterP*/FcfsArbiterP$1$ResourceCmd$deferRequest(/*CC2420RadioC.CmdTransmitC.ResourceC.ResourceUSARTP*/MSP430ResourceUSART0P$7$ID);
#line 63
}
#line 63
# 622 "/Users/jingyuancheng/tinyos/moteiv/tos/lib/CC2420Radio/CC2420RadioM.nc"
static inline void CC2420RadioM$BackoffAlarm32khz$fired(void )
#line 622
{
  uint8_t currentstate;

  /* atomic removed: atomic calls only */
#line 624
  currentstate = CC2420RadioM$stateRadio;

  switch (CC2420RadioM$stateTimer) {
      case CC2420RadioM$TIMER_INITIAL: 
        CC2420RadioM$CmdTransmit$deferRequest();
      break;
      case CC2420RadioM$TIMER_BACKOFF: 
        CC2420RadioM$CmdTryToSend$deferRequest();
      break;
      case CC2420RadioM$TIMER_ACK: 
        if (currentstate == CC2420RadioM$POST_TX_STATE) {
            /* atomic removed: atomic calls only */




            {
              CC2420RadioM$txbufptr->ack = 0;
              CC2420RadioM$stateRadio = CC2420RadioM$POST_TX_ACK_STATE;
            }
            if (!CC2420RadioM$PacketSent$postTask()) {
              CC2420RadioM$sendFailedAsync();
              }
          }
#line 647
      break;
      case CC2420RadioM$TIMER_SFD: 






        CC2420RadioM$SFD$disable();

      CC2420RadioM$SFD$enableCapture(TRUE);

      CC2420RadioM$sendFailedAsync();
      break;
    }
}

# 64 "/Users/jingyuancheng/tinyos/moteiv/tos/lib/timer/Alarm.nc"
inline static void /*CC2420RadioC.BackoffAlarm32khzC.MSP430Alarm*/MSP430AlarmC$1$Alarm$fired(void ){
#line 64
  CC2420RadioM$BackoffAlarm32khz$fired();
#line 64
}
#line 64
# 61 "/Users/jingyuancheng/tinyos/moteiv/tos/platform/msp430/timer/MSP430AlarmC.nc"
static inline void /*CC2420RadioC.BackoffAlarm32khzC.MSP430Alarm*/MSP430AlarmC$1$MSP430Compare$fired(void )
{
  /*CC2420RadioC.BackoffAlarm32khzC.MSP430Alarm*/MSP430AlarmC$1$MSP430TimerControl$disableEvents();
  /*CC2420RadioC.BackoffAlarm32khzC.MSP430Alarm*/MSP430AlarmC$1$Alarm$fired();
}

# 34 "/Users/jingyuancheng/tinyos/moteiv/tos/platform/msp430/timer/MSP430Compare.nc"
inline static void /*MSP430TimerC.MSP430TimerB3*/MSP430TimerCapComM$6$Compare$fired(void ){
#line 34
  /*CC2420RadioC.BackoffAlarm32khzC.MSP430Alarm*/MSP430AlarmC$1$MSP430Compare$fired();
#line 34
}
#line 34
# 138 "/Users/jingyuancheng/tinyos/moteiv/tos/platform/msp430/timer/MSP430TimerCapComM.nc"
static inline uint16_t /*MSP430TimerC.MSP430TimerB3*/MSP430TimerCapComM$6$Capture$getEvent(void )
{
  return * (volatile uint16_t *)408U;
}

#line 176
static inline void /*MSP430TimerC.MSP430TimerB3*/MSP430TimerCapComM$6$Capture$default$captured(uint16_t n)
{
}

# 74 "/Users/jingyuancheng/tinyos/moteiv/tos/platform/msp430/timer/MSP430Capture.nc"
inline static void /*MSP430TimerC.MSP430TimerB3*/MSP430TimerCapComM$6$Capture$captured(uint16_t time){
#line 74
  /*MSP430TimerC.MSP430TimerB3*/MSP430TimerCapComM$6$Capture$default$captured(time);
#line 74
}
#line 74
# 46 "/Users/jingyuancheng/tinyos/moteiv/tos/platform/msp430/timer/MSP430TimerCapComM.nc"
static inline /*MSP430TimerC.MSP430TimerB3*/MSP430TimerCapComM$6$CC_t /*MSP430TimerC.MSP430TimerB3*/MSP430TimerCapComM$6$int2CC(uint16_t x)
#line 46
{
#line 46
  union /*MSP430TimerC.MSP430TimerB3*/MSP430TimerCapComM$6$__nesc_unnamed4392 {
#line 46
    uint16_t f;
#line 46
    /*MSP430TimerC.MSP430TimerB3*/MSP430TimerCapComM$6$CC_t t;
  } 
#line 46
  c = { .f = x };

#line 46
  return c.t;
}

#line 73
static inline /*MSP430TimerC.MSP430TimerB3*/MSP430TimerCapComM$6$CC_t /*MSP430TimerC.MSP430TimerB3*/MSP430TimerCapComM$6$Control$getControl(void )
{
  return /*MSP430TimerC.MSP430TimerB3*/MSP430TimerCapComM$6$int2CC(* (volatile uint16_t *)392U);
}

#line 168
static inline void /*MSP430TimerC.MSP430TimerB3*/MSP430TimerCapComM$6$Event$fired(void )
{
  if (/*MSP430TimerC.MSP430TimerB3*/MSP430TimerCapComM$6$Control$getControl().cap) {
    /*MSP430TimerC.MSP430TimerB3*/MSP430TimerCapComM$6$Capture$captured(/*MSP430TimerC.MSP430TimerB3*/MSP430TimerCapComM$6$Capture$getEvent());
    }
  else {
#line 173
    /*MSP430TimerC.MSP430TimerB3*/MSP430TimerCapComM$6$Compare$fired();
    }
}

#line 123
static inline void /*MSP430TimerC.MSP430TimerB4*/MSP430TimerCapComM$7$Control$disableEvents(void )
{
  * (volatile uint16_t *)394U &= ~0x0010;
}

# 39 "/Users/jingyuancheng/tinyos/moteiv/tos/platform/msp430/timer/MSP430TimerControl.nc"
inline static void /*CC2420SyncAlwaysOnC.AlarmC.MSP430Alarm*/MSP430AlarmC$2$MSP430TimerControl$disableEvents(void ){
#line 39
  /*MSP430TimerC.MSP430TimerB4*/MSP430TimerCapComM$7$Control$disableEvents();
#line 39
}
#line 39
# 56 "/Users/jingyuancheng/tinyos/moteiv/tos/platform/msp430/timer/MSP430AlarmC.nc"
static inline void /*CC2420SyncAlwaysOnC.AlarmC.MSP430Alarm*/MSP430AlarmC$2$Alarm$stop(void )
{
  /*CC2420SyncAlwaysOnC.AlarmC.MSP430Alarm*/MSP430AlarmC$2$MSP430TimerControl$disableEvents();
}

# 60 "/Users/jingyuancheng/tinyos/moteiv/tos/lib/timer/Alarm.nc"
inline static void /*CC2420SyncAlwaysOnC.AlarmC.Transform*/TransformAlarmC$1$AlarmFrom$stop(void ){
#line 60
  /*CC2420SyncAlwaysOnC.AlarmC.MSP430Alarm*/MSP430AlarmC$2$Alarm$stop();
#line 60
}
#line 60
# 65 "/Users/jingyuancheng/tinyos/moteiv/tos/lib/timer/TransformAlarmC.nc"
static inline void /*CC2420SyncAlwaysOnC.AlarmC.Transform*/TransformAlarmC$1$Alarm$stop(void )
{
  /*CC2420SyncAlwaysOnC.AlarmC.Transform*/TransformAlarmC$1$AlarmFrom$stop();
}

# 60 "/Users/jingyuancheng/tinyos/moteiv/tos/lib/timer/Alarm.nc"
inline static void /*CC2420SyncAlwaysOnC.DualAlarmC*/VirtualizeAlarmC$0$AlarmFrom$stop(void ){
#line 60
  /*CC2420SyncAlwaysOnC.AlarmC.Transform*/TransformAlarmC$1$Alarm$stop();
#line 60
}
#line 60
# 100 "/Users/jingyuancheng/tinyos/moteiv/tos/lib/timer/TransformAlarmC.nc"
static inline void /*CC2420SyncAlwaysOnC.AlarmC.Transform*/TransformAlarmC$1$Alarm$startAt(/*CC2420SyncAlwaysOnC.AlarmC.Transform*/TransformAlarmC$1$to_size_type t0, /*CC2420SyncAlwaysOnC.AlarmC.Transform*/TransformAlarmC$1$to_size_type dt)
{
  /* atomic removed: atomic calls only */
  {
    /*CC2420SyncAlwaysOnC.AlarmC.Transform*/TransformAlarmC$1$m_t0 = t0;
    /*CC2420SyncAlwaysOnC.AlarmC.Transform*/TransformAlarmC$1$m_dt = dt;
    /*CC2420SyncAlwaysOnC.AlarmC.Transform*/TransformAlarmC$1$set_alarm();
  }
}

# 88 "/Users/jingyuancheng/tinyos/moteiv/tos/lib/timer/Alarm.nc"
inline static void /*CC2420SyncAlwaysOnC.DualAlarmC*/VirtualizeAlarmC$0$AlarmFrom$startAt(/*CC2420SyncAlwaysOnC.DualAlarmC*/VirtualizeAlarmC$0$AlarmFrom$size_type t0, /*CC2420SyncAlwaysOnC.DualAlarmC*/VirtualizeAlarmC$0$AlarmFrom$size_type dt){
#line 88
  /*CC2420SyncAlwaysOnC.AlarmC.Transform*/TransformAlarmC$1$Alarm$startAt(t0, dt);
#line 88
}
#line 88
# 52 "/Users/jingyuancheng/tinyos/moteiv/tos/lib/timer/Counter.nc"
inline static /*CC2420SyncAlwaysOnC.AlarmC.Transform*/TransformAlarmC$1$Counter$size_type /*CC2420SyncAlwaysOnC.AlarmC.Transform*/TransformAlarmC$1$Counter$get(void ){
#line 52
  unsigned long __nesc_result;
#line 52

#line 52
  __nesc_result = /*Counter32khzC.Transform*/TransformCounterC$1$Counter$get();
#line 52

#line 52
  return __nesc_result;
#line 52
}
#line 52
# 49 "/Users/jingyuancheng/tinyos/moteiv/tos/lib/timer/TransformAlarmC.nc"
static inline /*CC2420SyncAlwaysOnC.AlarmC.Transform*/TransformAlarmC$1$to_size_type /*CC2420SyncAlwaysOnC.AlarmC.Transform*/TransformAlarmC$1$Alarm$getNow(void )
{
  return /*CC2420SyncAlwaysOnC.AlarmC.Transform*/TransformAlarmC$1$Counter$get();
}

# 93 "/Users/jingyuancheng/tinyos/moteiv/tos/lib/timer/Alarm.nc"
inline static /*CC2420SyncAlwaysOnC.DualAlarmC*/VirtualizeAlarmC$0$AlarmFrom$size_type /*CC2420SyncAlwaysOnC.DualAlarmC*/VirtualizeAlarmC$0$AlarmFrom$getNow(void ){
#line 93
  unsigned long __nesc_result;
#line 93

#line 93
  __nesc_result = /*CC2420SyncAlwaysOnC.AlarmC.Transform*/TransformAlarmC$1$Alarm$getNow();
#line 93

#line 93
  return __nesc_result;
#line 93
}
#line 93
# 86 "/Users/jingyuancheng/tinyos/moteiv/tos/lib/timer/VirtualizeAlarmC.nc"
static inline void /*CC2420SyncAlwaysOnC.DualAlarmC*/VirtualizeAlarmC$0$setNextAlarm(void )
#line 86
{
  if (! /*CC2420SyncAlwaysOnC.DualAlarmC*/VirtualizeAlarmC$0$m.is_signaling) {








      const /*CC2420SyncAlwaysOnC.DualAlarmC*/VirtualizeAlarmC$0$size_type now = /*CC2420SyncAlwaysOnC.DualAlarmC*/VirtualizeAlarmC$0$AlarmFrom$getNow();
      const /*CC2420SyncAlwaysOnC.DualAlarmC*/VirtualizeAlarmC$0$alarm_t *pEnd = /*CC2420SyncAlwaysOnC.DualAlarmC*/VirtualizeAlarmC$0$m.alarm + /*CC2420SyncAlwaysOnC.DualAlarmC*/VirtualizeAlarmC$0$NUM_ALARMS;
      bool isset = FALSE;
      /*CC2420SyncAlwaysOnC.DualAlarmC*/VirtualizeAlarmC$0$alarm_t *p = /*CC2420SyncAlwaysOnC.DualAlarmC*/VirtualizeAlarmC$0$m.alarm;
      bool *pset = /*CC2420SyncAlwaysOnC.DualAlarmC*/VirtualizeAlarmC$0$m.isset;
      /*CC2420SyncAlwaysOnC.DualAlarmC*/VirtualizeAlarmC$0$size_type dt = (/*CC2420SyncAlwaysOnC.DualAlarmC*/VirtualizeAlarmC$0$size_type )0 - (/*CC2420SyncAlwaysOnC.DualAlarmC*/VirtualizeAlarmC$0$size_type )1;

      for (; p != pEnd; p++, pset++) {
          if (*pset) {
              /*CC2420SyncAlwaysOnC.DualAlarmC*/VirtualizeAlarmC$0$size_type elapsed = now - p->t0;

#line 106
              if (p->dt <= elapsed) {
                  p->t0 += p->dt;
                  p->dt = 0;
                }
              else {
                  p->t0 = now;
                  p->dt -= elapsed;
                }

              if (p->dt <= dt) {
                  dt = p->dt;
                  isset = TRUE;
                }
            }
        }

      if (isset) {




          if (dt & ((/*CC2420SyncAlwaysOnC.DualAlarmC*/VirtualizeAlarmC$0$size_type )1 << (8 * sizeof(/*CC2420SyncAlwaysOnC.DualAlarmC*/VirtualizeAlarmC$0$size_type ) - 1))) {
            dt >>= 1;
            }
          /*CC2420SyncAlwaysOnC.DualAlarmC*/VirtualizeAlarmC$0$AlarmFrom$startAt(now, dt);
        }
      else {
          /*CC2420SyncAlwaysOnC.DualAlarmC*/VirtualizeAlarmC$0$AlarmFrom$stop();
        }
    }
}

# 187 "/Users/jingyuancheng/tinyos/moteiv/tos/lib/sp/cc2420/CC2420AlwaysOnM.nc"
static inline void CC2420AlwaysOnM$AlarmStop$fired(void )
#line 187
{
}

#line 186
static inline void CC2420AlwaysOnM$AlarmStart$fired(void )
#line 186
{
}

# 196 "/Users/jingyuancheng/tinyos/moteiv/tos/lib/timer/VirtualizeAlarmC.nc"
static inline void /*CC2420SyncAlwaysOnC.DualAlarmC*/VirtualizeAlarmC$0$Alarm$default$fired(uint8_t id)
#line 196
{
}

# 64 "/Users/jingyuancheng/tinyos/moteiv/tos/lib/timer/Alarm.nc"
inline static void /*CC2420SyncAlwaysOnC.DualAlarmC*/VirtualizeAlarmC$0$Alarm$fired(uint8_t arg_0x102b44b20){
#line 64
  switch (arg_0x102b44b20) {
#line 64
    case 0:
#line 64
      CC2420AlwaysOnM$AlarmStart$fired();
#line 64
      break;
#line 64
    case 1:
#line 64
      CC2420AlwaysOnM$AlarmStop$fired();
#line 64
      break;
#line 64
    default:
#line 64
      /*CC2420SyncAlwaysOnC.DualAlarmC*/VirtualizeAlarmC$0$Alarm$default$fired(arg_0x102b44b20);
#line 64
      break;
#line 64
    }
#line 64
}
#line 64
# 138 "/Users/jingyuancheng/tinyos/moteiv/tos/lib/timer/VirtualizeAlarmC.nc"
static inline void /*CC2420SyncAlwaysOnC.DualAlarmC*/VirtualizeAlarmC$0$signalAlarms(void )
#line 138
{
  uint8_t id;

  /*CC2420SyncAlwaysOnC.DualAlarmC*/VirtualizeAlarmC$0$m.is_signaling = TRUE;

  for (id = 0; id < /*CC2420SyncAlwaysOnC.DualAlarmC*/VirtualizeAlarmC$0$NUM_ALARMS; id++) {
      if (/*CC2420SyncAlwaysOnC.DualAlarmC*/VirtualizeAlarmC$0$m.isset[id]) {
          /*CC2420SyncAlwaysOnC.DualAlarmC*/VirtualizeAlarmC$0$size_type elapsed = /*CC2420SyncAlwaysOnC.DualAlarmC*/VirtualizeAlarmC$0$AlarmFrom$getNow() - /*CC2420SyncAlwaysOnC.DualAlarmC*/VirtualizeAlarmC$0$m.alarm[id].t0;

#line 146
          if (/*CC2420SyncAlwaysOnC.DualAlarmC*/VirtualizeAlarmC$0$m.alarm[id].dt <= elapsed) {
              /*CC2420SyncAlwaysOnC.DualAlarmC*/VirtualizeAlarmC$0$m.isset[id] = FALSE;
              /*CC2420SyncAlwaysOnC.DualAlarmC*/VirtualizeAlarmC$0$Alarm$fired(id);
            }
        }
    }

  /*CC2420SyncAlwaysOnC.DualAlarmC*/VirtualizeAlarmC$0$m.is_signaling = FALSE;
}











static inline void /*CC2420SyncAlwaysOnC.DualAlarmC*/VirtualizeAlarmC$0$AlarmFrom$fired(void )
#line 166
{
  /* atomic removed: atomic calls only */
#line 167
  {
    /*CC2420SyncAlwaysOnC.DualAlarmC*/VirtualizeAlarmC$0$signalAlarms();
    /*CC2420SyncAlwaysOnC.DualAlarmC*/VirtualizeAlarmC$0$setNextAlarm();
  }
}

# 64 "/Users/jingyuancheng/tinyos/moteiv/tos/lib/timer/Alarm.nc"
inline static void /*CC2420SyncAlwaysOnC.AlarmC.Transform*/TransformAlarmC$1$Alarm$fired(void ){
#line 64
  /*CC2420SyncAlwaysOnC.DualAlarmC*/VirtualizeAlarmC$0$AlarmFrom$fired();
#line 64
}
#line 64
# 115 "/Users/jingyuancheng/tinyos/moteiv/tos/lib/timer/TransformAlarmC.nc"
static inline void /*CC2420SyncAlwaysOnC.AlarmC.Transform*/TransformAlarmC$1$AlarmFrom$fired(void )
{
  /* atomic removed: atomic calls only */
  {
    if (/*CC2420SyncAlwaysOnC.AlarmC.Transform*/TransformAlarmC$1$m_dt == 0) 
      {
        /*CC2420SyncAlwaysOnC.AlarmC.Transform*/TransformAlarmC$1$Alarm$fired();
      }
    else 
      {
        /*CC2420SyncAlwaysOnC.AlarmC.Transform*/TransformAlarmC$1$set_alarm();
      }
  }
}

# 64 "/Users/jingyuancheng/tinyos/moteiv/tos/lib/timer/Alarm.nc"
inline static void /*CC2420SyncAlwaysOnC.AlarmC.MSP430Alarm*/MSP430AlarmC$2$Alarm$fired(void ){
#line 64
  /*CC2420SyncAlwaysOnC.AlarmC.Transform*/TransformAlarmC$1$AlarmFrom$fired();
#line 64
}
#line 64
# 61 "/Users/jingyuancheng/tinyos/moteiv/tos/platform/msp430/timer/MSP430AlarmC.nc"
static inline void /*CC2420SyncAlwaysOnC.AlarmC.MSP430Alarm*/MSP430AlarmC$2$MSP430Compare$fired(void )
{
  /*CC2420SyncAlwaysOnC.AlarmC.MSP430Alarm*/MSP430AlarmC$2$MSP430TimerControl$disableEvents();
  /*CC2420SyncAlwaysOnC.AlarmC.MSP430Alarm*/MSP430AlarmC$2$Alarm$fired();
}

# 34 "/Users/jingyuancheng/tinyos/moteiv/tos/platform/msp430/timer/MSP430Compare.nc"
inline static void /*MSP430TimerC.MSP430TimerB4*/MSP430TimerCapComM$7$Compare$fired(void ){
#line 34
  /*CC2420SyncAlwaysOnC.AlarmC.MSP430Alarm*/MSP430AlarmC$2$MSP430Compare$fired();
#line 34
}
#line 34
# 138 "/Users/jingyuancheng/tinyos/moteiv/tos/platform/msp430/timer/MSP430TimerCapComM.nc"
static inline uint16_t /*MSP430TimerC.MSP430TimerB4*/MSP430TimerCapComM$7$Capture$getEvent(void )
{
  return * (volatile uint16_t *)410U;
}

#line 176
static inline void /*MSP430TimerC.MSP430TimerB4*/MSP430TimerCapComM$7$Capture$default$captured(uint16_t n)
{
}

# 74 "/Users/jingyuancheng/tinyos/moteiv/tos/platform/msp430/timer/MSP430Capture.nc"
inline static void /*MSP430TimerC.MSP430TimerB4*/MSP430TimerCapComM$7$Capture$captured(uint16_t time){
#line 74
  /*MSP430TimerC.MSP430TimerB4*/MSP430TimerCapComM$7$Capture$default$captured(time);
#line 74
}
#line 74
# 46 "/Users/jingyuancheng/tinyos/moteiv/tos/platform/msp430/timer/MSP430TimerCapComM.nc"
static inline /*MSP430TimerC.MSP430TimerB4*/MSP430TimerCapComM$7$CC_t /*MSP430TimerC.MSP430TimerB4*/MSP430TimerCapComM$7$int2CC(uint16_t x)
#line 46
{
#line 46
  union /*MSP430TimerC.MSP430TimerB4*/MSP430TimerCapComM$7$__nesc_unnamed4393 {
#line 46
    uint16_t f;
#line 46
    /*MSP430TimerC.MSP430TimerB4*/MSP430TimerCapComM$7$CC_t t;
  } 
#line 46
  c = { .f = x };

#line 46
  return c.t;
}

#line 73
static inline /*MSP430TimerC.MSP430TimerB4*/MSP430TimerCapComM$7$CC_t /*MSP430TimerC.MSP430TimerB4*/MSP430TimerCapComM$7$Control$getControl(void )
{
  return /*MSP430TimerC.MSP430TimerB4*/MSP430TimerCapComM$7$int2CC(* (volatile uint16_t *)394U);
}

#line 168
static inline void /*MSP430TimerC.MSP430TimerB4*/MSP430TimerCapComM$7$Event$fired(void )
{
  if (/*MSP430TimerC.MSP430TimerB4*/MSP430TimerCapComM$7$Control$getControl().cap) {
    /*MSP430TimerC.MSP430TimerB4*/MSP430TimerCapComM$7$Capture$captured(/*MSP430TimerC.MSP430TimerB4*/MSP430TimerCapComM$7$Capture$getEvent());
    }
  else {
#line 173
    /*MSP430TimerC.MSP430TimerB4*/MSP430TimerCapComM$7$Compare$fired();
    }
}

# 88 "/Users/jingyuancheng/tinyos/moteiv/tos/lib/timer/Alarm.nc"
inline static void /*CC2420SyncAlwaysOnC.AlarmC.Transform*/TransformAlarmC$1$AlarmFrom$startAt(/*CC2420SyncAlwaysOnC.AlarmC.Transform*/TransformAlarmC$1$AlarmFrom$size_type t0, /*CC2420SyncAlwaysOnC.AlarmC.Transform*/TransformAlarmC$1$AlarmFrom$size_type dt){
#line 88
  /*CC2420SyncAlwaysOnC.AlarmC.MSP430Alarm*/MSP430AlarmC$2$Alarm$startAt(t0, dt);
#line 88
}
#line 88
# 30 "/Users/jingyuancheng/tinyos/moteiv/tos/platform/msp430/timer/MSP430Timer.nc"
inline static uint16_t /*CC2420SyncAlwaysOnC.AlarmC.MSP430Alarm*/MSP430AlarmC$2$MSP430Timer$get(void ){
#line 30
  unsigned int __nesc_result;
#line 30

#line 30
  __nesc_result = /*MSP430TimerC.MSP430TimerB*/MSP430TimerM$1$Timer$get();
#line 30

#line 30
  return __nesc_result;
#line 30
}
#line 30
inline static uint16_t /*MSP430TimerC.MSP430TimerB4*/MSP430TimerCapComM$7$Timer$get(void ){
#line 30
  unsigned int __nesc_result;
#line 30

#line 30
  __nesc_result = /*MSP430TimerC.MSP430TimerB*/MSP430TimerM$1$Timer$get();
#line 30

#line 30
  return __nesc_result;
#line 30
}
#line 30
# 153 "/Users/jingyuancheng/tinyos/moteiv/tos/platform/msp430/timer/MSP430TimerCapComM.nc"
static inline void /*MSP430TimerC.MSP430TimerB4*/MSP430TimerCapComM$7$Compare$setEventFromNow(uint16_t x)
{
  * (volatile uint16_t *)410U = /*MSP430TimerC.MSP430TimerB4*/MSP430TimerCapComM$7$Timer$get() + x;
}

# 32 "/Users/jingyuancheng/tinyos/moteiv/tos/platform/msp430/timer/MSP430Compare.nc"
inline static void /*CC2420SyncAlwaysOnC.AlarmC.MSP430Alarm*/MSP430AlarmC$2$MSP430Compare$setEventFromNow(uint16_t delta){
#line 32
  /*MSP430TimerC.MSP430TimerB4*/MSP430TimerCapComM$7$Compare$setEventFromNow(delta);
#line 32
}
#line 32
# 143 "/Users/jingyuancheng/tinyos/moteiv/tos/platform/msp430/timer/MSP430TimerCapComM.nc"
static inline void /*MSP430TimerC.MSP430TimerB4*/MSP430TimerCapComM$7$Compare$setEvent(uint16_t x)
{
  * (volatile uint16_t *)410U = x;
}

# 30 "/Users/jingyuancheng/tinyos/moteiv/tos/platform/msp430/timer/MSP430Compare.nc"
inline static void /*CC2420SyncAlwaysOnC.AlarmC.MSP430Alarm*/MSP430AlarmC$2$MSP430Compare$setEvent(uint16_t time){
#line 30
  /*MSP430TimerC.MSP430TimerB4*/MSP430TimerCapComM$7$Compare$setEvent(time);
#line 30
}
#line 30
# 83 "/Users/jingyuancheng/tinyos/moteiv/tos/platform/msp430/timer/MSP430TimerCapComM.nc"
static inline void /*MSP430TimerC.MSP430TimerB4*/MSP430TimerCapComM$7$Control$clearPendingInterrupt(void )
{
  * (volatile uint16_t *)394U &= ~0x0001;
}

# 32 "/Users/jingyuancheng/tinyos/moteiv/tos/platform/msp430/timer/MSP430TimerControl.nc"
inline static void /*CC2420SyncAlwaysOnC.AlarmC.MSP430Alarm*/MSP430AlarmC$2$MSP430TimerControl$clearPendingInterrupt(void ){
#line 32
  /*MSP430TimerC.MSP430TimerB4*/MSP430TimerCapComM$7$Control$clearPendingInterrupt();
#line 32
}
#line 32
# 118 "/Users/jingyuancheng/tinyos/moteiv/tos/platform/msp430/timer/MSP430TimerCapComM.nc"
static inline void /*MSP430TimerC.MSP430TimerB4*/MSP430TimerCapComM$7$Control$enableEvents(void )
{
  * (volatile uint16_t *)394U |= 0x0010;
}

# 38 "/Users/jingyuancheng/tinyos/moteiv/tos/platform/msp430/timer/MSP430TimerControl.nc"
inline static void /*CC2420SyncAlwaysOnC.AlarmC.MSP430Alarm*/MSP430AlarmC$2$MSP430TimerControl$enableEvents(void ){
#line 38
  /*MSP430TimerC.MSP430TimerB4*/MSP430TimerCapComM$7$Control$enableEvents();
#line 38
}
#line 38
# 180 "/Users/jingyuancheng/tinyos/moteiv/tos/platform/msp430/timer/MSP430TimerCapComM.nc"
static inline void /*MSP430TimerC.MSP430TimerB5*/MSP430TimerCapComM$8$Compare$default$fired(void )
{
}

# 34 "/Users/jingyuancheng/tinyos/moteiv/tos/platform/msp430/timer/MSP430Compare.nc"
inline static void /*MSP430TimerC.MSP430TimerB5*/MSP430TimerCapComM$8$Compare$fired(void ){
#line 34
  /*MSP430TimerC.MSP430TimerB5*/MSP430TimerCapComM$8$Compare$default$fired();
#line 34
}
#line 34
# 138 "/Users/jingyuancheng/tinyos/moteiv/tos/platform/msp430/timer/MSP430TimerCapComM.nc"
static inline uint16_t /*MSP430TimerC.MSP430TimerB5*/MSP430TimerCapComM$8$Capture$getEvent(void )
{
  return * (volatile uint16_t *)412U;
}

#line 176
static inline void /*MSP430TimerC.MSP430TimerB5*/MSP430TimerCapComM$8$Capture$default$captured(uint16_t n)
{
}

# 74 "/Users/jingyuancheng/tinyos/moteiv/tos/platform/msp430/timer/MSP430Capture.nc"
inline static void /*MSP430TimerC.MSP430TimerB5*/MSP430TimerCapComM$8$Capture$captured(uint16_t time){
#line 74
  /*MSP430TimerC.MSP430TimerB5*/MSP430TimerCapComM$8$Capture$default$captured(time);
#line 74
}
#line 74
# 46 "/Users/jingyuancheng/tinyos/moteiv/tos/platform/msp430/timer/MSP430TimerCapComM.nc"
static inline /*MSP430TimerC.MSP430TimerB5*/MSP430TimerCapComM$8$CC_t /*MSP430TimerC.MSP430TimerB5*/MSP430TimerCapComM$8$int2CC(uint16_t x)
#line 46
{
#line 46
  union /*MSP430TimerC.MSP430TimerB5*/MSP430TimerCapComM$8$__nesc_unnamed4394 {
#line 46
    uint16_t f;
#line 46
    /*MSP430TimerC.MSP430TimerB5*/MSP430TimerCapComM$8$CC_t t;
  } 
#line 46
  c = { .f = x };

#line 46
  return c.t;
}

#line 73
static inline /*MSP430TimerC.MSP430TimerB5*/MSP430TimerCapComM$8$CC_t /*MSP430TimerC.MSP430TimerB5*/MSP430TimerCapComM$8$Control$getControl(void )
{
  return /*MSP430TimerC.MSP430TimerB5*/MSP430TimerCapComM$8$int2CC(* (volatile uint16_t *)396U);
}

#line 168
static inline void /*MSP430TimerC.MSP430TimerB5*/MSP430TimerCapComM$8$Event$fired(void )
{
  if (/*MSP430TimerC.MSP430TimerB5*/MSP430TimerCapComM$8$Control$getControl().cap) {
    /*MSP430TimerC.MSP430TimerB5*/MSP430TimerCapComM$8$Capture$captured(/*MSP430TimerC.MSP430TimerB5*/MSP430TimerCapComM$8$Capture$getEvent());
    }
  else {
#line 173
    /*MSP430TimerC.MSP430TimerB5*/MSP430TimerCapComM$8$Compare$fired();
    }
}




static inline void /*MSP430TimerC.MSP430TimerB6*/MSP430TimerCapComM$9$Compare$default$fired(void )
{
}

# 34 "/Users/jingyuancheng/tinyos/moteiv/tos/platform/msp430/timer/MSP430Compare.nc"
inline static void /*MSP430TimerC.MSP430TimerB6*/MSP430TimerCapComM$9$Compare$fired(void ){
#line 34
  /*MSP430TimerC.MSP430TimerB6*/MSP430TimerCapComM$9$Compare$default$fired();
#line 34
}
#line 34
# 138 "/Users/jingyuancheng/tinyos/moteiv/tos/platform/msp430/timer/MSP430TimerCapComM.nc"
static inline uint16_t /*MSP430TimerC.MSP430TimerB6*/MSP430TimerCapComM$9$Capture$getEvent(void )
{
  return * (volatile uint16_t *)414U;
}

#line 176
static inline void /*MSP430TimerC.MSP430TimerB6*/MSP430TimerCapComM$9$Capture$default$captured(uint16_t n)
{
}

# 74 "/Users/jingyuancheng/tinyos/moteiv/tos/platform/msp430/timer/MSP430Capture.nc"
inline static void /*MSP430TimerC.MSP430TimerB6*/MSP430TimerCapComM$9$Capture$captured(uint16_t time){
#line 74
  /*MSP430TimerC.MSP430TimerB6*/MSP430TimerCapComM$9$Capture$default$captured(time);
#line 74
}
#line 74
# 46 "/Users/jingyuancheng/tinyos/moteiv/tos/platform/msp430/timer/MSP430TimerCapComM.nc"
static inline /*MSP430TimerC.MSP430TimerB6*/MSP430TimerCapComM$9$CC_t /*MSP430TimerC.MSP430TimerB6*/MSP430TimerCapComM$9$int2CC(uint16_t x)
#line 46
{
#line 46
  union /*MSP430TimerC.MSP430TimerB6*/MSP430TimerCapComM$9$__nesc_unnamed4395 {
#line 46
    uint16_t f;
#line 46
    /*MSP430TimerC.MSP430TimerB6*/MSP430TimerCapComM$9$CC_t t;
  } 
#line 46
  c = { .f = x };

#line 46
  return c.t;
}

#line 73
static inline /*MSP430TimerC.MSP430TimerB6*/MSP430TimerCapComM$9$CC_t /*MSP430TimerC.MSP430TimerB6*/MSP430TimerCapComM$9$Control$getControl(void )
{
  return /*MSP430TimerC.MSP430TimerB6*/MSP430TimerCapComM$9$int2CC(* (volatile uint16_t *)398U);
}

#line 168
static inline void /*MSP430TimerC.MSP430TimerB6*/MSP430TimerCapComM$9$Event$fired(void )
{
  if (/*MSP430TimerC.MSP430TimerB6*/MSP430TimerCapComM$9$Control$getControl().cap) {
    /*MSP430TimerC.MSP430TimerB6*/MSP430TimerCapComM$9$Capture$captured(/*MSP430TimerC.MSP430TimerB6*/MSP430TimerCapComM$9$Capture$getEvent());
    }
  else {
#line 173
    /*MSP430TimerC.MSP430TimerB6*/MSP430TimerCapComM$9$Compare$fired();
    }
}

# 119 "/Users/jingyuancheng/tinyos/moteiv/tos/platform/msp430/timer/MSP430TimerM.nc"
static inline void /*MSP430TimerC.MSP430TimerB*/MSP430TimerM$1$VectorTimerX1$fired(void )
{
  uint8_t n = * (volatile uint16_t *)286U;

#line 122
  /*MSP430TimerC.MSP430TimerB*/MSP430TimerM$1$Event$fired(n >> 1);
}

# 4 "/Users/jingyuancheng/tinyos/moteiv/tos/platform/msp430/timer/MSP430TimerEvent.nc"
inline static void MSP430TimerCommonM$VectorTimerB1$fired(void ){
#line 4
  /*MSP430TimerC.MSP430TimerB*/MSP430TimerM$1$VectorTimerX1$fired();
#line 4
}
#line 4
# 145 "/Users/jingyuancheng/tinyos/moteiv/tos/lib/sched/SchedulerBasicP.nc"
static inline void SchedulerBasicP$Scheduler$init(void )
#line 145
{
  /* atomic removed: atomic calls only */
#line 146
  {
    uint8_t *n = SchedulerBasicP$m_next;

#line 148
    while (n != SchedulerBasicP$m_next + sizeof SchedulerBasicP$m_next) 
      * n++ = SchedulerBasicP$NONE;
    SchedulerBasicP$m_head = SchedulerBasicP$NONE;
    SchedulerBasicP$m_tail = SchedulerBasicP$NONE;
  }
}

# 41 "/Users/jingyuancheng/tinyos/moteiv/tos/lib/sched/Scheduler.nc"
inline static void MainP$Scheduler$init(void ){
#line 41
  SchedulerBasicP$Scheduler$init();
#line 41
}
#line 41
# 125 "/Users/jingyuancheng/tinyos/moteiv/tos/system/tos.h"
static inline result_t rcombine(result_t r1, result_t r2)



{
  return r1 == FAIL ? FAIL : r2;
}

# 95 "/Users/jingyuancheng/tinyos/moteiv/tos/platform/msp430/adc/HPLADC12M.nc"
static inline void HPLADC12M$HPLADC12$setIEFlags(uint16_t mask)
#line 95
{
#line 95
  HPLADC12M$ADC12IE = mask;
}

# 55 "/Users/jingyuancheng/tinyos/moteiv/tos/platform/msp430/adc/HPLADC12.nc"
inline static void MSP430ADC12M$HPLADC12$setIEFlags(uint16_t mask){
#line 55
  HPLADC12M$HPLADC12$setIEFlags(mask);
#line 55
}
#line 55
# 115 "/Users/jingyuancheng/tinyos/moteiv/tos/platform/msp430/adc/HPLADC12M.nc"
static inline void HPLADC12M$HPLADC12$disableConversion(void )
#line 115
{
#line 115
  HPLADC12M$ADC12CTL0 &= ~0x0002;
}

# 80 "/Users/jingyuancheng/tinyos/moteiv/tos/platform/msp430/adc/HPLADC12.nc"
inline static void MSP430ADC12M$HPLADC12$disableConversion(void ){
#line 80
  HPLADC12M$HPLADC12$disableConversion();
#line 80
}
#line 80
# 67 "/Users/jingyuancheng/tinyos/moteiv/tos/platform/msp430/adc/MSP430ADC12M.nc"
static inline result_t MSP430ADC12M$StdControl$init(void )
{
  MSP430ADC12M$cmode = ADC_IDLE;
  MSP430ADC12M$reserved = ADC_IDLE;
  MSP430ADC12M$vrefWait = FALSE;
  MSP430ADC12M$HPLADC12$disableConversion();
  MSP430ADC12M$HPLADC12$setIEFlags(0x0000);
  return SUCCESS;
}

# 50 "ADC8IO14P.nc"
static inline result_t ADC8IO14P$StdControl$init(void )
{

  ADCMsg_t *body = (ADCMsg_t *)ADC8IO14P$m_msg.data;

#line 54
  ADC8IO14P$m_count = 0;
  ADC8IO14P$m_sending = FALSE;

  body->data[0] = 0x00;
  body->data[1] = 0x11;
  body->data[2] = 0x22;
  body->data[3] = 0x33;
  body->data[4] = 0x44;
  body->data[5] = 0x55;
  body->data[6] = 0x66;
  body->data[7] = 0x77;
  body->count = ADC8IO14P$m_count;

  return SUCCESS;
}

# 63 "/Users/jingyuancheng/tinyos/moteiv/tinyos-1.x/tos/interfaces/StdControl.nc"
inline static result_t MainP$StdControl$init(void ){
#line 63
  unsigned char __nesc_result;
#line 63

#line 63
  __nesc_result = ADC8IO14P$StdControl$init();
#line 63
  __nesc_result = rcombine(__nesc_result, MSP430ADC12M$StdControl$init());
#line 63

#line 63
  return __nesc_result;
#line 63
}
#line 63
# 94 "/Users/jingyuancheng/tinyos/moteiv/tos/lib/sched/MainP.nc"
static inline result_t MainP$MainInit$default$init(uint8_t id)
#line 94
{
  return FAIL;
}

# 46 "/Users/jingyuancheng/tinyos/moteiv/tos/lib/sched/Init.nc"
inline static result_t MainP$MainInit$init(uint8_t arg_0x101d28020){
#line 46
  unsigned char __nesc_result;
#line 46

#line 46
    __nesc_result = MainP$MainInit$default$init(arg_0x101d28020);
#line 46

#line 46
  return __nesc_result;
#line 46
}
#line 46
# 164 "/Users/jingyuancheng/tinyos/moteiv/tos/system/tos.h"
static inline void *nmemset(void *to, int val, size_t n)
{
  char *cto = to;

  while (n--) * cto++ = val;

  return to;
}

# 59 "/Users/jingyuancheng/tinyos/moteiv/tos/lib/timer/VirtualizeTimerC.nc"
static inline /*HalTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC$0$error_t /*HalTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC$0$Init$init(void )
{
  nmemset(/*HalTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC$0$m_timers, 0, sizeof /*HalTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC$0$m_timers);
  nmemset(/*HalTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC$0$m_flags, 0, sizeof /*HalTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC$0$m_flags);
  return SUCCESS;
}

# 45 "/Users/jingyuancheng/tinyos/moteiv/tos/platform/msp430/timer/MSP430TimerCapComM.nc"
static inline uint16_t /*MSP430TimerC.MSP430TimerB2*/MSP430TimerCapComM$5$CC2int(/*MSP430TimerC.MSP430TimerB2*/MSP430TimerCapComM$5$CC_t x)
#line 45
{
#line 45
  union /*MSP430TimerC.MSP430TimerB2*/MSP430TimerCapComM$5$__nesc_unnamed4396 {
#line 45
    /*MSP430TimerC.MSP430TimerB2*/MSP430TimerCapComM$5$CC_t f;
#line 45
    uint16_t t;
  } 
#line 45
  c = { .f = x };

#line 45
  return c.t;
}

static inline uint16_t /*MSP430TimerC.MSP430TimerB2*/MSP430TimerCapComM$5$compareControl(void )
{
  /*MSP430TimerC.MSP430TimerB2*/MSP430TimerCapComM$5$CC_t x = { 
  .cm = 1, 
  .ccis = 0, 
  .clld = 0, 
  .cap = 0, 
  .ccie = 0 };

  return /*MSP430TimerC.MSP430TimerB2*/MSP430TimerCapComM$5$CC2int(x);
}

#line 93
static inline void /*MSP430TimerC.MSP430TimerB2*/MSP430TimerCapComM$5$Control$setControlAsCompare(void )
{
  * (volatile uint16_t *)390U = /*MSP430TimerC.MSP430TimerB2*/MSP430TimerCapComM$5$compareControl();
}

# 35 "/Users/jingyuancheng/tinyos/moteiv/tos/platform/msp430/timer/MSP430TimerControl.nc"
inline static void /*HalTimerMilliC.AlarmMilliC.MSP430Alarm*/MSP430AlarmC$0$MSP430TimerControl$setControlAsCompare(void ){
#line 35
  /*MSP430TimerC.MSP430TimerB2*/MSP430TimerCapComM$5$Control$setControlAsCompare();
#line 35
}
#line 35
# 41 "/Users/jingyuancheng/tinyos/moteiv/tos/platform/msp430/timer/MSP430AlarmC.nc"
static inline /*HalTimerMilliC.AlarmMilliC.MSP430Alarm*/MSP430AlarmC$0$error_t /*HalTimerMilliC.AlarmMilliC.MSP430Alarm*/MSP430AlarmC$0$Init$init(void )
{
  /*HalTimerMilliC.AlarmMilliC.MSP430Alarm*/MSP430AlarmC$0$MSP430TimerControl$disableEvents();
  /*HalTimerMilliC.AlarmMilliC.MSP430Alarm*/MSP430AlarmC$0$MSP430TimerControl$setControlAsCompare();
  return SUCCESS;
}

# 60 "/Users/jingyuancheng/tinyos/moteiv/tinyos-1.x/tos/system/UARTM.nc"
static inline result_t UARTM$Control$init(void )
#line 60
{
  {
  }
#line 61
  ;
  /* atomic removed: atomic calls only */
#line 62
  {
    UARTM$state = FALSE;
  }
  return SUCCESS;
}

# 63 "/Users/jingyuancheng/tinyos/moteiv/tinyos-1.x/tos/interfaces/StdControl.nc"
inline static result_t FramerP$ByteControl$init(void ){
#line 63
  unsigned char __nesc_result;
#line 63

#line 63
  __nesc_result = UARTM$Control$init();
#line 63

#line 63
  return __nesc_result;
#line 63
}
#line 63
# 307 "/Users/jingyuancheng/tinyos/moteiv/tos/platform/tmote/FramerP.nc"
static inline result_t FramerP$StdControl$init(void )
#line 307
{
  FramerP$HDLCInitialize();
  return FramerP$ByteControl$init();
}

# 55 "/Users/jingyuancheng/tinyos/moteiv/tos/platform/tmote/util/uartdetect/UartPresenceM.nc"
static inline result_t UartPresenceM$StdControl$init(void )
#line 55
{
  return SUCCESS;
}

# 63 "/Users/jingyuancheng/tinyos/moteiv/tos/lib/resource/ResourceCmd.nc"
inline static void CC2420ControlM$CmdSplitControlInit$deferRequest(void ){
#line 63
  /*MSP430ArbiterUSART0C.ArbiterC.ArbiterP*/FcfsArbiterP$1$ResourceCmd$deferRequest(/*CC2420RadioC.CmdSplitControlInitC.ResourceC.ResourceUSARTP*/MSP430ResourceUSART0P$1$ID);
#line 63
}
#line 63
# 175 "/Users/jingyuancheng/tinyos/moteiv/tos/lib/CC2420Radio/CC2420ControlM.nc"
static inline result_t CC2420ControlM$SplitControl$init(void )
#line 175
{

  uint8_t _state = FALSE;

  /* atomic removed: atomic calls only */
#line 179
  {
    if (CC2420ControlM$state == CC2420ControlM$IDLE_STATE) {
        CC2420ControlM$state = CC2420ControlM$INIT_STATE;
        _state = TRUE;
      }
  }
  if (!_state) {
    return FAIL;
    }
  CC2420ControlM$CmdSplitControlInit$deferRequest();
  return SUCCESS;
}

# 64 "/Users/jingyuancheng/tinyos/moteiv/tinyos-1.x/tos/interfaces/SplitControl.nc"
inline static result_t CC2420RadioM$CC2420SplitControl$init(void ){
#line 64
  unsigned char __nesc_result;
#line 64

#line 64
  __nesc_result = CC2420ControlM$SplitControl$init();
#line 64

#line 64
  return __nesc_result;
#line 64
}
#line 64
# 38 "/Users/jingyuancheng/tinyos/moteiv/tos/platform/msp430/RandomMLCG.nc"
static inline result_t RandomMLCG$Random$init(void )
#line 38
{
  /* atomic removed: atomic calls only */
#line 39
  RandomMLCG$seed = (uint32_t )(TOS_LOCAL_ADDRESS + 1);
  return SUCCESS;
}

# 57 "/Users/jingyuancheng/tinyos/moteiv/tinyos-1.x/tos/interfaces/Random.nc"
inline static result_t CC2420RadioM$Random$init(void ){
#line 57
  unsigned char __nesc_result;
#line 57

#line 57
  __nesc_result = RandomMLCG$Random$init();
#line 57

#line 57
  return __nesc_result;
#line 57
}
#line 57
# 45 "/Users/jingyuancheng/tinyos/moteiv/tos/platform/msp430/timer/MSP430TimerCapComM.nc"
static inline uint16_t /*MSP430TimerC.MSP430TimerB3*/MSP430TimerCapComM$6$CC2int(/*MSP430TimerC.MSP430TimerB3*/MSP430TimerCapComM$6$CC_t x)
#line 45
{
#line 45
  union /*MSP430TimerC.MSP430TimerB3*/MSP430TimerCapComM$6$__nesc_unnamed4397 {
#line 45
    /*MSP430TimerC.MSP430TimerB3*/MSP430TimerCapComM$6$CC_t f;
#line 45
    uint16_t t;
  } 
#line 45
  c = { .f = x };

#line 45
  return c.t;
}

static inline uint16_t /*MSP430TimerC.MSP430TimerB3*/MSP430TimerCapComM$6$compareControl(void )
{
  /*MSP430TimerC.MSP430TimerB3*/MSP430TimerCapComM$6$CC_t x = { 
  .cm = 1, 
  .ccis = 0, 
  .clld = 0, 
  .cap = 0, 
  .ccie = 0 };

  return /*MSP430TimerC.MSP430TimerB3*/MSP430TimerCapComM$6$CC2int(x);
}

#line 93
static inline void /*MSP430TimerC.MSP430TimerB3*/MSP430TimerCapComM$6$Control$setControlAsCompare(void )
{
  * (volatile uint16_t *)392U = /*MSP430TimerC.MSP430TimerB3*/MSP430TimerCapComM$6$compareControl();
}

# 35 "/Users/jingyuancheng/tinyos/moteiv/tos/platform/msp430/timer/MSP430TimerControl.nc"
inline static void /*CC2420RadioC.BackoffAlarm32khzC.MSP430Alarm*/MSP430AlarmC$1$MSP430TimerControl$setControlAsCompare(void ){
#line 35
  /*MSP430TimerC.MSP430TimerB3*/MSP430TimerCapComM$6$Control$setControlAsCompare();
#line 35
}
#line 35
# 41 "/Users/jingyuancheng/tinyos/moteiv/tos/platform/msp430/timer/MSP430AlarmC.nc"
static inline /*CC2420RadioC.BackoffAlarm32khzC.MSP430Alarm*/MSP430AlarmC$1$error_t /*CC2420RadioC.BackoffAlarm32khzC.MSP430Alarm*/MSP430AlarmC$1$Init$init(void )
{
  /*CC2420RadioC.BackoffAlarm32khzC.MSP430Alarm*/MSP430AlarmC$1$MSP430TimerControl$disableEvents();
  /*CC2420RadioC.BackoffAlarm32khzC.MSP430Alarm*/MSP430AlarmC$1$MSP430TimerControl$setControlAsCompare();
  return SUCCESS;
}

# 63 "/Users/jingyuancheng/tinyos/moteiv/tinyos-1.x/tos/interfaces/StdControl.nc"
inline static result_t CC2420RadioM$TimerControl$init(void ){
#line 63
  unsigned char __nesc_result;
#line 63

#line 63
  __nesc_result = /*CC2420RadioC.BackoffAlarm32khzC.MSP430Alarm*/MSP430AlarmC$1$Init$init();
#line 63

#line 63
  return __nesc_result;
#line 63
}
#line 63
# 285 "/Users/jingyuancheng/tinyos/moteiv/tos/lib/CC2420Radio/CC2420RadioM.nc"
static inline result_t CC2420RadioM$SplitControl$init(void )
#line 285
{
  /* atomic removed: atomic calls only */
  {
    CC2420RadioM$stateRadio = CC2420RadioM$DISABLED_STATE;
    CC2420RadioM$currentDSN = 0;
    CC2420RadioM$bPacketReceiving = FALSE;
    CC2420RadioM$rxbufptr = &CC2420RadioM$RxBuf;
    CC2420RadioM$rxbufptr->length = 0;
    CC2420RadioM$m_sfdReceiving = FALSE;
    CC2420RadioM$m_rxFifoCount = 0;
    cqueue_init(&CC2420RadioM$m_timestampQueue, CC2420RadioM$NUM_TIMESTAMPS);
  }

  CC2420RadioM$TimerControl$init();
  CC2420RadioM$Random$init();
  CC2420RadioM$LocalAddr = TOS_LOCAL_ADDRESS;
  return CC2420RadioM$CC2420SplitControl$init();
}

# 64 "/Users/jingyuancheng/tinyos/moteiv/tinyos-1.x/tos/interfaces/SplitControl.nc"
inline static result_t CC2420AlwaysOnM$RadioControl$init(void ){
#line 64
  unsigned char __nesc_result;
#line 64

#line 64
  __nesc_result = CC2420RadioM$SplitControl$init();
#line 64

#line 64
  return __nesc_result;
#line 64
}
#line 64
# 54 "/Users/jingyuancheng/tinyos/moteiv/tos/lib/sp/cc2420/CC2420AlwaysOnM.nc"
static inline result_t CC2420AlwaysOnM$StdControl$init(void )
#line 54
{
  return CC2420AlwaysOnM$RadioControl$init();
}

# 45 "/Users/jingyuancheng/tinyos/moteiv/tos/platform/msp430/timer/MSP430TimerCapComM.nc"
static inline uint16_t /*MSP430TimerC.MSP430TimerB4*/MSP430TimerCapComM$7$CC2int(/*MSP430TimerC.MSP430TimerB4*/MSP430TimerCapComM$7$CC_t x)
#line 45
{
#line 45
  union /*MSP430TimerC.MSP430TimerB4*/MSP430TimerCapComM$7$__nesc_unnamed4398 {
#line 45
    /*MSP430TimerC.MSP430TimerB4*/MSP430TimerCapComM$7$CC_t f;
#line 45
    uint16_t t;
  } 
#line 45
  c = { .f = x };

#line 45
  return c.t;
}

static inline uint16_t /*MSP430TimerC.MSP430TimerB4*/MSP430TimerCapComM$7$compareControl(void )
{
  /*MSP430TimerC.MSP430TimerB4*/MSP430TimerCapComM$7$CC_t x = { 
  .cm = 1, 
  .ccis = 0, 
  .clld = 0, 
  .cap = 0, 
  .ccie = 0 };

  return /*MSP430TimerC.MSP430TimerB4*/MSP430TimerCapComM$7$CC2int(x);
}

#line 93
static inline void /*MSP430TimerC.MSP430TimerB4*/MSP430TimerCapComM$7$Control$setControlAsCompare(void )
{
  * (volatile uint16_t *)394U = /*MSP430TimerC.MSP430TimerB4*/MSP430TimerCapComM$7$compareControl();
}

# 35 "/Users/jingyuancheng/tinyos/moteiv/tos/platform/msp430/timer/MSP430TimerControl.nc"
inline static void /*CC2420SyncAlwaysOnC.AlarmC.MSP430Alarm*/MSP430AlarmC$2$MSP430TimerControl$setControlAsCompare(void ){
#line 35
  /*MSP430TimerC.MSP430TimerB4*/MSP430TimerCapComM$7$Control$setControlAsCompare();
#line 35
}
#line 35
# 41 "/Users/jingyuancheng/tinyos/moteiv/tos/platform/msp430/timer/MSP430AlarmC.nc"
static inline /*CC2420SyncAlwaysOnC.AlarmC.MSP430Alarm*/MSP430AlarmC$2$error_t /*CC2420SyncAlwaysOnC.AlarmC.MSP430Alarm*/MSP430AlarmC$2$Init$init(void )
{
  /*CC2420SyncAlwaysOnC.AlarmC.MSP430Alarm*/MSP430AlarmC$2$MSP430TimerControl$disableEvents();
  /*CC2420SyncAlwaysOnC.AlarmC.MSP430Alarm*/MSP430AlarmC$2$MSP430TimerControl$setControlAsCompare();
  return SUCCESS;
}

# 73 "/Users/jingyuancheng/tinyos/moteiv/tos/lib/timer/VirtualizeAlarmC.nc"
static inline /*CC2420SyncAlwaysOnC.DualAlarmC*/VirtualizeAlarmC$0$error_t /*CC2420SyncAlwaysOnC.DualAlarmC*/VirtualizeAlarmC$0$Init$init(void )
#line 73
{
  nmemset(&/*CC2420SyncAlwaysOnC.DualAlarmC*/VirtualizeAlarmC$0$m, 0, sizeof /*CC2420SyncAlwaysOnC.DualAlarmC*/VirtualizeAlarmC$0$m);
  return SUCCESS;
}

# 81 "/Users/jingyuancheng/tinyos/moteiv/tos/lib/sched/MainP.nc"
static inline result_t MainP$MainStdControl$default$init(uint8_t id)
#line 81
{
  return MainP$MainInit$init(id);
}

# 63 "/Users/jingyuancheng/tinyos/moteiv/tinyos-1.x/tos/interfaces/StdControl.nc"
inline static result_t MainP$MainStdControl$init(uint8_t arg_0x101d29020){
#line 63
  unsigned char __nesc_result;
#line 63

#line 63
  switch (arg_0x101d29020) {
#line 63
    case /*MainTimerMilliC.MainControlC*/MainControlC$0$ID:
#line 63
      __nesc_result = /*HalTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC$0$Init$init();
#line 63
      __nesc_result = rcombine(__nesc_result, /*HalTimerMilliC.AlarmMilliC.MSP430Alarm*/MSP430AlarmC$0$Init$init());
#line 63
      break;
#line 63
    case /*MainUartPacketC.MainControlC*/MainControlC$1$ID:
#line 63
      __nesc_result = FramerP$StdControl$init();
#line 63
      break;
#line 63
    case /*MainUartPresenceC.MainControlC*/MainControlC$2$ID:
#line 63
      __nesc_result = UartPresenceM$StdControl$init();
#line 63
      break;
#line 63
    case /*MainSPC.MainControlC*/MainControlC$3$ID:
#line 63
      __nesc_result = /*CC2420SyncAlwaysOnC.DualAlarmC*/VirtualizeAlarmC$0$Init$init();
#line 63
      __nesc_result = rcombine(__nesc_result, CC2420AlwaysOnM$StdControl$init());
#line 63
      __nesc_result = rcombine(__nesc_result, /*CC2420SyncAlwaysOnC.AlarmC.MSP430Alarm*/MSP430AlarmC$2$Init$init());
#line 63
      break;
#line 63
    default:
#line 63
      __nesc_result = MainP$MainStdControl$default$init(arg_0x101d29020);
#line 63
      break;
#line 63
    }
#line 63

#line 63
  return __nesc_result;
#line 63
}
#line 63
# 65 "/Users/jingyuancheng/tinyos/moteiv/tos/lib/sched/MainP.nc"
static inline result_t MainP$MainSplitControl$default$init(uint8_t id)
#line 65
{
  MainP$MainStdControl$init(id);
  return FAIL;
}

# 64 "/Users/jingyuancheng/tinyos/moteiv/tinyos-1.x/tos/interfaces/SplitControl.nc"
inline static result_t MainP$MainSplitControl$init(uint8_t arg_0x101d2caa8){
#line 64
  unsigned char __nesc_result;
#line 64

#line 64
    __nesc_result = MainP$MainSplitControl$default$init(arg_0x101d2caa8);
#line 64

#line 64
  return __nesc_result;
#line 64
}
#line 64
# 48 "/Users/jingyuancheng/tinyos/moteiv/tos/lib/resource/FcfsArbiterP.nc"
static inline result_t /*MSP430ArbiterTimerAC.ArbiterC.ArbiterP*/FcfsArbiterP$0$Init$init(void )
#line 48
{
  /* atomic removed: atomic calls only */
#line 49
  {
    rqueue_init(/*MSP430ArbiterTimerAC.ArbiterC.ArbiterP*/FcfsArbiterP$0$queue(), /*MSP430ArbiterTimerAC.ArbiterC.ArbiterP*/FcfsArbiterP$0$COUNT);
    /*MSP430ArbiterTimerAC.ArbiterC.ArbiterP*/FcfsArbiterP$0$m_urgentCount = 0;
    /*MSP430ArbiterTimerAC.ArbiterC.ArbiterP*/FcfsArbiterP$0$m_granted = RESOURCE_NONE;
    {
      unsigned char __nesc_temp = 
#line 53
      SUCCESS;

#line 53
      return __nesc_temp;
    }
  }
}

#line 43
static inline ReservedQueue_t */*MSP430ArbiterUSART0C.ArbiterC.ArbiterP*/FcfsArbiterP$1$queue(void )
#line 43
{
  return (ReservedQueue_t *)&/*MSP430ArbiterUSART0C.ArbiterC.ArbiterP*/FcfsArbiterP$1$m_queue;
}


static inline result_t /*MSP430ArbiterUSART0C.ArbiterC.ArbiterP*/FcfsArbiterP$1$Init$init(void )
#line 48
{
  /* atomic removed: atomic calls only */
#line 49
  {
    rqueue_init(/*MSP430ArbiterUSART0C.ArbiterC.ArbiterP*/FcfsArbiterP$1$queue(), /*MSP430ArbiterUSART0C.ArbiterC.ArbiterP*/FcfsArbiterP$1$COUNT);
    /*MSP430ArbiterUSART0C.ArbiterC.ArbiterP*/FcfsArbiterP$1$m_urgentCount = 0;
    /*MSP430ArbiterUSART0C.ArbiterC.ArbiterP*/FcfsArbiterP$1$m_granted = RESOURCE_NONE;
    {
      unsigned char __nesc_temp = 
#line 53
      SUCCESS;

#line 53
      return __nesc_temp;
    }
  }
}

# 46 "/Users/jingyuancheng/tinyos/moteiv/tos/lib/sched/Init.nc"
inline static result_t PlatformP$ArbiterInits$init(void ){
#line 46
  unsigned char __nesc_result;
#line 46

#line 46
  __nesc_result = /*MSP430ArbiterUSART0C.ArbiterC.ArbiterP*/FcfsArbiterP$1$Init$init();
#line 46
  __nesc_result = rcombine(__nesc_result, /*MSP430ArbiterTimerAC.ArbiterC.ArbiterP*/FcfsArbiterP$0$Init$init());
#line 46

#line 46
  return __nesc_result;
#line 46
}
#line 46
# 238 "/Users/jingyuancheng/tinyos/moteiv/tos/platform/msp430/timer/MSP430ClockM.nc"
static inline result_t MSP430ClockM$Init$start(void )
#line 238
{
#line 238
  return SUCCESS;
}

# 70 "/Users/jingyuancheng/tinyos/moteiv/tinyos-1.x/tos/interfaces/StdControl.nc"
inline static result_t HPLInitM$MSP430ClockControl$start(void ){
#line 70
  unsigned char __nesc_result;
#line 70

#line 70
  __nesc_result = MSP430ClockM$Init$start();
#line 70

#line 70
  return __nesc_result;
#line 70
}
#line 70
# 139 "/Users/jingyuancheng/tinyos/moteiv/tos/platform/msp430/timer/MSP430ClockM.nc"
static inline void MSP430ClockM$startTimerB(void )
{

  MSP430ClockM$TBCTL = 0x0020 | (MSP430ClockM$TBCTL & ~(0x0020 | 0x0010));
}

#line 127
static inline void MSP430ClockM$startTimerA(void )
{

  MSP430ClockM$TA0CTL = 0x0020 | (MSP430ClockM$TA0CTL & ~(0x0020 | 0x0010));
}

#line 96
static inline void MSP430ClockM$MSP430ClockInit$defaultInitTimerB(void )
{
  TBR = 0;









  MSP430ClockM$TBCTL = 0x0100 | 0x0002;
}











static inline void MSP430ClockM$MSP430ClockInit$default$initTimerB(void )
{
  MSP430ClockM$MSP430ClockInit$defaultInitTimerB();
}

# 29 "/Users/jingyuancheng/tinyos/moteiv/tos/platform/msp430/timer/MSP430ClockInit.nc"
inline static void MSP430ClockM$MSP430ClockInit$initTimerB(void ){
#line 29
  MSP430ClockM$MSP430ClockInit$default$initTimerB();
#line 29
}
#line 29
# 81 "/Users/jingyuancheng/tinyos/moteiv/tos/platform/msp430/timer/MSP430ClockM.nc"
static inline void MSP430ClockM$MSP430ClockInit$defaultInitTimerA(void )
{
  TA0R = 0;









  MSP430ClockM$TA0CTL = 0;
}

#line 116
static inline void MSP430ClockM$MSP430ClockInit$default$initTimerA(void )
{
  MSP430ClockM$MSP430ClockInit$defaultInitTimerA();
}

# 28 "/Users/jingyuancheng/tinyos/moteiv/tos/platform/msp430/timer/MSP430ClockInit.nc"
inline static void MSP430ClockM$MSP430ClockInit$initTimerA(void ){
#line 28
  MSP430ClockM$MSP430ClockInit$default$initTimerA();
#line 28
}
#line 28
# 54 "/Users/jingyuancheng/tinyos/moteiv/tos/platform/msp430/timer/MSP430ClockM.nc"
static inline void MSP430ClockM$MSP430ClockInit$defaultInitClocks(void )
{





  BCSCTL1 = 0x80 | (BCSCTL1 & ((0x04 | 0x02) | 0x01));
#line 74
  BCSCTL2 = 0x04;



  MSP430ClockM$IE1 &= ~(1 << 1);
}

#line 111
static inline void MSP430ClockM$MSP430ClockInit$default$initClocks(void )
{
  MSP430ClockM$MSP430ClockInit$defaultInitClocks();
}

# 27 "/Users/jingyuancheng/tinyos/moteiv/tos/platform/msp430/timer/MSP430ClockInit.nc"
inline static void MSP430ClockM$MSP430ClockInit$initClocks(void ){
#line 27
  MSP430ClockM$MSP430ClockInit$default$initClocks();
#line 27
}
#line 27
# 157 "/Users/jingyuancheng/tinyos/moteiv/tos/platform/msp430/timer/MSP430ClockM.nc"
static inline uint16_t MSP430ClockM$test_calib_busywait_delta(int calib)
{
  int8_t aclk_count = 2;
  uint16_t dco_prev = 0;
  uint16_t dco_curr = 0;

  MSP430ClockM$set_dco_calib(calib);

  while (aclk_count-- > 0) 
    {
      TBCCR0 = TBR + MSP430ClockM$ACLK_CALIB_PERIOD;
      TBCCTL0 &= ~0x0001;
      while ((TBCCTL0 & 0x0001) == 0) ;
      dco_prev = dco_curr;
      dco_curr = TA0R;
    }

  return dco_curr - dco_prev;
}




static inline void MSP430ClockM$busyCalibrateDCO(void )
{

  int calib;
  int step;



  MSP430ClockM$TA0CTL = 0x0200 | 0x0020;
  MSP430ClockM$TBCTL = 0x0100 | 0x0020;
  BCSCTL1 = 0x80 | 0x04;



  BCSCTL2 = 0;

  TBCCTL0 = 0x4000;






  for (calib = 0, step = 0x800; step != 0; step >>= 1) 
    {

      if (MSP430ClockM$test_calib_busywait_delta(calib | step) <= MSP430ClockM$TARGET_DCO_DELTA) {
        calib |= step;
        }
    }

  if ((calib & 0x0e0) == 0x0e0) {
    calib &= ~0x01f;
    }
  MSP430ClockM$set_dco_calib(calib);
}

static inline MSP430ClockM$error_t MSP430ClockM$Init$init(void )
{

  MSP430ClockM$TA0CTL = 0x0004;
  MSP430ClockM$TA0IV = 0;
  MSP430ClockM$TBCTL = 0x0004;
  MSP430ClockM$TBIV = 0;
  /* atomic removed: atomic calls only */

  {
    MSP430ClockM$busyCalibrateDCO();
    MSP430ClockM$MSP430ClockInit$initClocks();
    MSP430ClockM$MSP430ClockInit$initTimerA();
    MSP430ClockM$MSP430ClockInit$initTimerB();
    MSP430ClockM$startTimerA();
    MSP430ClockM$startTimerB();
  }

  return SUCCESS;
}

# 63 "/Users/jingyuancheng/tinyos/moteiv/tinyos-1.x/tos/interfaces/StdControl.nc"
inline static result_t HPLInitM$MSP430ClockControl$init(void ){
#line 63
  unsigned char __nesc_result;
#line 63

#line 63
  __nesc_result = MSP430ClockM$Init$init();
#line 63

#line 63
  return __nesc_result;
#line 63
}
#line 63
# 38 "/Users/jingyuancheng/tinyos/moteiv/tos/platform/tmote/hardware.h"
static inline void TOSH_SET_SIMO0_PIN()
#line 38
{
#line 38
  static volatile uint8_t r __asm ("0x0019");

#line 38
  r |= 1 << 1;
}

#line 39
static inline void TOSH_SET_UCLK0_PIN()
#line 39
{
#line 39
  static volatile uint8_t r __asm ("0x0019");

#line 39
  r |= 1 << 3;
}

#line 76
static inline void TOSH_SET_FLASH_CS_PIN()
#line 76
{
#line 76
  static volatile uint8_t r __asm ("0x001D");

#line 76
  r |= 1 << 4;
}

#line 77
static inline void TOSH_SET_FLASH_HOLD_PIN()
#line 77
{
#line 77
  static volatile uint8_t r __asm ("0x001D");

#line 77
  r |= 1 << 7;
}

#line 39
static inline void TOSH_CLR_UCLK0_PIN()
#line 39
{
#line 39
  static volatile uint8_t r __asm ("0x0019");

#line 39
  r &= ~(1 << 3);
}

#line 76
static inline void TOSH_CLR_FLASH_CS_PIN()
#line 76
{
#line 76
  static volatile uint8_t r __asm ("0x001D");

#line 76
  r &= ~(1 << 4);
}

# 161 "/Users/jingyuancheng/tinyos/moteiv/tos/platform/msp430/msp430hardware.h"
static __inline void TOSH_wait(void )
{
   __asm volatile ("nop"); __asm volatile ("nop");}

# 76 "/Users/jingyuancheng/tinyos/moteiv/tos/platform/tmote/hardware.h"
static inline void TOSH_MAKE_FLASH_CS_OUTPUT()
#line 76
{
#line 76
  static volatile uint8_t r __asm ("0x001E");

#line 76
  r |= 1 << 4;
}

#line 77
static inline void TOSH_MAKE_FLASH_HOLD_OUTPUT()
#line 77
{
#line 77
  static volatile uint8_t r __asm ("0x001E");

#line 77
  r |= 1 << 7;
}

#line 39
static inline void TOSH_MAKE_UCLK0_OUTPUT()
#line 39
{
#line 39
  static volatile uint8_t r __asm ("0x001A");

#line 39
  r |= 1 << 3;
}

#line 38
static inline void TOSH_MAKE_SIMO0_OUTPUT()
#line 38
{
#line 38
  static volatile uint8_t r __asm ("0x001A");

#line 38
  r |= 1 << 1;
}

#line 99
static inline void TOSH_FLASH_M25P_DP()
#line 99
{

  TOSH_MAKE_SIMO0_OUTPUT();
  TOSH_MAKE_UCLK0_OUTPUT();
  TOSH_MAKE_FLASH_HOLD_OUTPUT();
  TOSH_MAKE_FLASH_CS_OUTPUT();
  TOSH_SET_FLASH_HOLD_PIN();
  TOSH_SET_FLASH_CS_PIN();

  TOSH_wait();


  TOSH_CLR_FLASH_CS_PIN();
  TOSH_CLR_UCLK0_PIN();

  TOSH_FLASH_M25P_DP_bit(TRUE);
  TOSH_FLASH_M25P_DP_bit(FALSE);
  TOSH_FLASH_M25P_DP_bit(TRUE);
  TOSH_FLASH_M25P_DP_bit(TRUE);
  TOSH_FLASH_M25P_DP_bit(TRUE);
  TOSH_FLASH_M25P_DP_bit(FALSE);
  TOSH_FLASH_M25P_DP_bit(FALSE);
  TOSH_FLASH_M25P_DP_bit(TRUE);





  TOSH_SET_FLASH_HOLD_PIN();
  TOSH_SET_FLASH_CS_PIN();
  TOSH_SET_UCLK0_PIN();
  TOSH_SET_SIMO0_PIN();
}

# 174 "/Users/jingyuancheng/tinyos/moteiv/tos/platform/msp430/msp430hardware.h"
static __inline void TOSH_uwait(uint16_t u)
{
  uint16_t i;

#line 177
  if (u < 500) {
    for (i = 2; i < u; i++) {
         __asm volatile ("nop\n\t"
        "nop\n\t"
        "nop\n\t"
        "nop\n\t");}
    }
  else {

    for (i = 0; i < u; i++) {
         __asm volatile ("nop\n\t"
        "nop\n\t"
        "nop\n\t"
        "nop\n\t");}
    }
}

# 279 "/Users/jingyuancheng/tinyos/moteiv/tos/platform/tmote/hardware.h"
static inline void TOSH_SET_PIN_DIRECTIONS(void )
{
  /* atomic removed: atomic calls only */

  {


    P1SEL = ((((((((MSP430_INIT_P10 & MSP430_INIT_PIN_SELBIT ? 1 : 0) << 0) | ((
    MSP430_INIT_P11 & MSP430_INIT_PIN_SELBIT ? 1 : 0) << 1)) | ((
    MSP430_INIT_P12 & MSP430_INIT_PIN_SELBIT ? 1 : 0) << 2)) | ((
    MSP430_INIT_P13 & MSP430_INIT_PIN_SELBIT ? 1 : 0) << 3)) | ((
    MSP430_INIT_P14 & MSP430_INIT_PIN_SELBIT ? 1 : 0) << 4)) | ((
    MSP430_INIT_P15 & MSP430_INIT_PIN_SELBIT ? 1 : 0) << 5)) | ((
    MSP430_INIT_P16 & MSP430_INIT_PIN_SELBIT ? 1 : 0) << 6)) | ((
    MSP430_INIT_P17 & MSP430_INIT_PIN_SELBIT ? 1 : 0) << 7);

    P1DIR = ((((((((MSP430_INIT_P10 & MSP430_INIT_PIN_DIRBIT ? 1 : 0) << 0) | ((
    MSP430_INIT_P11 & MSP430_INIT_PIN_DIRBIT ? 1 : 0) << 1)) | ((
    MSP430_INIT_P12 & MSP430_INIT_PIN_DIRBIT ? 1 : 0) << 2)) | ((
    MSP430_INIT_P13 & MSP430_INIT_PIN_DIRBIT ? 1 : 0) << 3)) | ((
    MSP430_INIT_P14 & MSP430_INIT_PIN_DIRBIT ? 1 : 0) << 4)) | ((
    MSP430_INIT_P15 & MSP430_INIT_PIN_DIRBIT ? 1 : 0) << 5)) | ((
    MSP430_INIT_P16 & MSP430_INIT_PIN_DIRBIT ? 1 : 0) << 6)) | ((
    MSP430_INIT_P17 & MSP430_INIT_PIN_DIRBIT ? 1 : 0) << 7);

    P1OUT = ((((((((MSP430_INIT_P10 & MSP430_INIT_PIN_OUTBIT ? 1 : 0) << 0) | ((
    MSP430_INIT_P11 & MSP430_INIT_PIN_OUTBIT ? 1 : 0) << 1)) | ((
    MSP430_INIT_P12 & MSP430_INIT_PIN_OUTBIT ? 1 : 0) << 2)) | ((
    MSP430_INIT_P13 & MSP430_INIT_PIN_OUTBIT ? 1 : 0) << 3)) | ((
    MSP430_INIT_P14 & MSP430_INIT_PIN_OUTBIT ? 1 : 0) << 4)) | ((
    MSP430_INIT_P15 & MSP430_INIT_PIN_OUTBIT ? 1 : 0) << 5)) | ((
    MSP430_INIT_P16 & MSP430_INIT_PIN_OUTBIT ? 1 : 0) << 6)) | ((
    MSP430_INIT_P17 & MSP430_INIT_PIN_OUTBIT ? 1 : 0) << 7);


    P2SEL = ((((((((MSP430_INIT_P20 & MSP430_INIT_PIN_SELBIT ? 1 : 0) << 0) | ((
    MSP430_INIT_P21 & MSP430_INIT_PIN_SELBIT ? 1 : 0) << 1)) | ((
    MSP430_INIT_P22 & MSP430_INIT_PIN_SELBIT ? 1 : 0) << 2)) | ((
    MSP430_INIT_P23 & MSP430_INIT_PIN_SELBIT ? 1 : 0) << 3)) | ((
    MSP430_INIT_P24 & MSP430_INIT_PIN_SELBIT ? 1 : 0) << 4)) | ((
    MSP430_INIT_P25 & MSP430_INIT_PIN_SELBIT ? 1 : 0) << 5)) | ((
    MSP430_INIT_P26 & MSP430_INIT_PIN_SELBIT ? 1 : 0) << 6)) | ((
    MSP430_INIT_P27 & MSP430_INIT_PIN_SELBIT ? 1 : 0) << 7);

    P2DIR = ((((((((MSP430_INIT_P20 & MSP430_INIT_PIN_DIRBIT ? 1 : 0) << 0) | ((
    MSP430_INIT_P21 & MSP430_INIT_PIN_DIRBIT ? 1 : 0) << 1)) | ((
    MSP430_INIT_P22 & MSP430_INIT_PIN_DIRBIT ? 1 : 0) << 2)) | ((
    MSP430_INIT_P23 & MSP430_INIT_PIN_DIRBIT ? 1 : 0) << 3)) | ((
    MSP430_INIT_P24 & MSP430_INIT_PIN_DIRBIT ? 1 : 0) << 4)) | ((
    MSP430_INIT_P25 & MSP430_INIT_PIN_DIRBIT ? 1 : 0) << 5)) | ((
    MSP430_INIT_P26 & MSP430_INIT_PIN_DIRBIT ? 1 : 0) << 6)) | ((
    MSP430_INIT_P27 & MSP430_INIT_PIN_DIRBIT ? 1 : 0) << 7);

    P2OUT = ((((((((MSP430_INIT_P20 & MSP430_INIT_PIN_OUTBIT ? 1 : 0) << 0) | ((
    MSP430_INIT_P21 & MSP430_INIT_PIN_OUTBIT ? 1 : 0) << 1)) | ((
    MSP430_INIT_P22 & MSP430_INIT_PIN_OUTBIT ? 1 : 0) << 2)) | ((
    MSP430_INIT_P23 & MSP430_INIT_PIN_OUTBIT ? 1 : 0) << 3)) | ((
    MSP430_INIT_P24 & MSP430_INIT_PIN_OUTBIT ? 1 : 0) << 4)) | ((
    MSP430_INIT_P25 & MSP430_INIT_PIN_OUTBIT ? 1 : 0) << 5)) | ((
    MSP430_INIT_P26 & MSP430_INIT_PIN_OUTBIT ? 1 : 0) << 6)) | ((
    MSP430_INIT_P27 & MSP430_INIT_PIN_OUTBIT ? 1 : 0) << 7);


    P3SEL = ((((((((MSP430_INIT_P30 & MSP430_INIT_PIN_SELBIT ? 1 : 0) << 0) | ((
    MSP430_INIT_P31 & MSP430_INIT_PIN_SELBIT ? 1 : 0) << 1)) | ((
    MSP430_INIT_P32 & MSP430_INIT_PIN_SELBIT ? 1 : 0) << 2)) | ((
    MSP430_INIT_P33 & MSP430_INIT_PIN_SELBIT ? 1 : 0) << 3)) | ((
    MSP430_INIT_P34 & MSP430_INIT_PIN_SELBIT ? 1 : 0) << 4)) | ((
    MSP430_INIT_P35 & MSP430_INIT_PIN_SELBIT ? 1 : 0) << 5)) | ((
    MSP430_INIT_P36 & MSP430_INIT_PIN_SELBIT ? 1 : 0) << 6)) | ((
    MSP430_INIT_P37 & MSP430_INIT_PIN_SELBIT ? 1 : 0) << 7);

    P3DIR = ((((((((MSP430_INIT_P30 & MSP430_INIT_PIN_DIRBIT ? 1 : 0) << 0) | ((
    MSP430_INIT_P31 & MSP430_INIT_PIN_DIRBIT ? 1 : 0) << 1)) | ((
    MSP430_INIT_P32 & MSP430_INIT_PIN_DIRBIT ? 1 : 0) << 2)) | ((
    MSP430_INIT_P33 & MSP430_INIT_PIN_DIRBIT ? 1 : 0) << 3)) | ((
    MSP430_INIT_P34 & MSP430_INIT_PIN_DIRBIT ? 1 : 0) << 4)) | ((
    MSP430_INIT_P35 & MSP430_INIT_PIN_DIRBIT ? 1 : 0) << 5)) | ((
    MSP430_INIT_P36 & MSP430_INIT_PIN_DIRBIT ? 1 : 0) << 6)) | ((
    MSP430_INIT_P37 & MSP430_INIT_PIN_DIRBIT ? 1 : 0) << 7);

    P3OUT = ((((((((MSP430_INIT_P30 & MSP430_INIT_PIN_OUTBIT ? 1 : 0) << 0) | ((
    MSP430_INIT_P31 & MSP430_INIT_PIN_OUTBIT ? 1 : 0) << 1)) | ((
    MSP430_INIT_P32 & MSP430_INIT_PIN_OUTBIT ? 1 : 0) << 2)) | ((
    MSP430_INIT_P33 & MSP430_INIT_PIN_OUTBIT ? 1 : 0) << 3)) | ((
    MSP430_INIT_P34 & MSP430_INIT_PIN_OUTBIT ? 1 : 0) << 4)) | ((
    MSP430_INIT_P35 & MSP430_INIT_PIN_OUTBIT ? 1 : 0) << 5)) | ((
    MSP430_INIT_P36 & MSP430_INIT_PIN_OUTBIT ? 1 : 0) << 6)) | ((
    MSP430_INIT_P37 & MSP430_INIT_PIN_OUTBIT ? 1 : 0) << 7);


    P4SEL = ((((((((MSP430_INIT_P40 & MSP430_INIT_PIN_SELBIT ? 1 : 0) << 0) | ((
    MSP430_INIT_P41 & MSP430_INIT_PIN_SELBIT ? 1 : 0) << 1)) | ((
    MSP430_INIT_P42 & MSP430_INIT_PIN_SELBIT ? 1 : 0) << 2)) | ((
    MSP430_INIT_P43 & MSP430_INIT_PIN_SELBIT ? 1 : 0) << 3)) | ((
    MSP430_INIT_P44 & MSP430_INIT_PIN_SELBIT ? 1 : 0) << 4)) | ((
    MSP430_INIT_P45 & MSP430_INIT_PIN_SELBIT ? 1 : 0) << 5)) | ((
    MSP430_INIT_P46 & MSP430_INIT_PIN_SELBIT ? 1 : 0) << 6)) | ((
    MSP430_INIT_P47 & MSP430_INIT_PIN_SELBIT ? 1 : 0) << 7);

    P4DIR = ((((((((MSP430_INIT_P40 & MSP430_INIT_PIN_DIRBIT ? 1 : 0) << 0) | ((
    MSP430_INIT_P41 & MSP430_INIT_PIN_DIRBIT ? 1 : 0) << 1)) | ((
    MSP430_INIT_P42 & MSP430_INIT_PIN_DIRBIT ? 1 : 0) << 2)) | ((
    MSP430_INIT_P43 & MSP430_INIT_PIN_DIRBIT ? 1 : 0) << 3)) | ((
    MSP430_INIT_P44 & MSP430_INIT_PIN_DIRBIT ? 1 : 0) << 4)) | ((
    MSP430_INIT_P45 & MSP430_INIT_PIN_DIRBIT ? 1 : 0) << 5)) | ((
    MSP430_INIT_P46 & MSP430_INIT_PIN_DIRBIT ? 1 : 0) << 6)) | ((
    MSP430_INIT_P47 & MSP430_INIT_PIN_DIRBIT ? 1 : 0) << 7);

    P4OUT = ((((((((MSP430_INIT_P40 & MSP430_INIT_PIN_OUTBIT ? 1 : 0) << 0) | ((
    MSP430_INIT_P41 & MSP430_INIT_PIN_OUTBIT ? 1 : 0) << 1)) | ((
    MSP430_INIT_P42 & MSP430_INIT_PIN_OUTBIT ? 1 : 0) << 2)) | ((
    MSP430_INIT_P43 & MSP430_INIT_PIN_OUTBIT ? 1 : 0) << 3)) | ((
    MSP430_INIT_P44 & MSP430_INIT_PIN_OUTBIT ? 1 : 0) << 4)) | ((
    MSP430_INIT_P45 & MSP430_INIT_PIN_OUTBIT ? 1 : 0) << 5)) | ((
    MSP430_INIT_P46 & MSP430_INIT_PIN_OUTBIT ? 1 : 0) << 6)) | ((
    MSP430_INIT_P47 & MSP430_INIT_PIN_OUTBIT ? 1 : 0) << 7);


    P5SEL = ((((((((MSP430_INIT_P50 & MSP430_INIT_PIN_SELBIT ? 1 : 0) << 0) | ((
    MSP430_INIT_P51 & MSP430_INIT_PIN_SELBIT ? 1 : 0) << 1)) | ((
    MSP430_INIT_P52 & MSP430_INIT_PIN_SELBIT ? 1 : 0) << 2)) | ((
    MSP430_INIT_P53 & MSP430_INIT_PIN_SELBIT ? 1 : 0) << 3)) | ((
    MSP430_INIT_P54 & MSP430_INIT_PIN_SELBIT ? 1 : 0) << 4)) | ((
    MSP430_INIT_P55 & MSP430_INIT_PIN_SELBIT ? 1 : 0) << 5)) | ((
    MSP430_INIT_P56 & MSP430_INIT_PIN_SELBIT ? 1 : 0) << 6)) | ((
    MSP430_INIT_P57 & MSP430_INIT_PIN_SELBIT ? 1 : 0) << 7);

    P5DIR = ((((((((MSP430_INIT_P50 & MSP430_INIT_PIN_DIRBIT ? 1 : 0) << 0) | ((
    MSP430_INIT_P51 & MSP430_INIT_PIN_DIRBIT ? 1 : 0) << 1)) | ((
    MSP430_INIT_P52 & MSP430_INIT_PIN_DIRBIT ? 1 : 0) << 2)) | ((
    MSP430_INIT_P53 & MSP430_INIT_PIN_DIRBIT ? 1 : 0) << 3)) | ((
    MSP430_INIT_P54 & MSP430_INIT_PIN_DIRBIT ? 1 : 0) << 4)) | ((
    MSP430_INIT_P55 & MSP430_INIT_PIN_DIRBIT ? 1 : 0) << 5)) | ((
    MSP430_INIT_P56 & MSP430_INIT_PIN_DIRBIT ? 1 : 0) << 6)) | ((
    MSP430_INIT_P57 & MSP430_INIT_PIN_DIRBIT ? 1 : 0) << 7);

    P5OUT = ((((((((MSP430_INIT_P50 & MSP430_INIT_PIN_OUTBIT ? 1 : 0) << 0) | ((
    MSP430_INIT_P51 & MSP430_INIT_PIN_OUTBIT ? 1 : 0) << 1)) | ((
    MSP430_INIT_P52 & MSP430_INIT_PIN_OUTBIT ? 1 : 0) << 2)) | ((
    MSP430_INIT_P53 & MSP430_INIT_PIN_OUTBIT ? 1 : 0) << 3)) | ((
    MSP430_INIT_P54 & MSP430_INIT_PIN_OUTBIT ? 1 : 0) << 4)) | ((
    MSP430_INIT_P55 & MSP430_INIT_PIN_OUTBIT ? 1 : 0) << 5)) | ((
    MSP430_INIT_P56 & MSP430_INIT_PIN_OUTBIT ? 1 : 0) << 6)) | ((
    MSP430_INIT_P57 & MSP430_INIT_PIN_OUTBIT ? 1 : 0) << 7);


    P6SEL = ((((((((MSP430_INIT_P60 & MSP430_INIT_PIN_SELBIT ? 1 : 0) << 0) | ((
    MSP430_INIT_P61 & MSP430_INIT_PIN_SELBIT ? 1 : 0) << 1)) | ((
    MSP430_INIT_P62 & MSP430_INIT_PIN_SELBIT ? 1 : 0) << 2)) | ((
    MSP430_INIT_P63 & MSP430_INIT_PIN_SELBIT ? 1 : 0) << 3)) | ((
    MSP430_INIT_P64 & MSP430_INIT_PIN_SELBIT ? 1 : 0) << 4)) | ((
    MSP430_INIT_P65 & MSP430_INIT_PIN_SELBIT ? 1 : 0) << 5)) | ((
    MSP430_INIT_P66 & MSP430_INIT_PIN_SELBIT ? 1 : 0) << 6)) | ((
    MSP430_INIT_P67 & MSP430_INIT_PIN_SELBIT ? 1 : 0) << 7);

    P6DIR = ((((((((MSP430_INIT_P60 & MSP430_INIT_PIN_DIRBIT ? 1 : 0) << 0) | ((
    MSP430_INIT_P61 & MSP430_INIT_PIN_DIRBIT ? 1 : 0) << 1)) | ((
    MSP430_INIT_P62 & MSP430_INIT_PIN_DIRBIT ? 1 : 0) << 2)) | ((
    MSP430_INIT_P63 & MSP430_INIT_PIN_DIRBIT ? 1 : 0) << 3)) | ((
    MSP430_INIT_P64 & MSP430_INIT_PIN_DIRBIT ? 1 : 0) << 4)) | ((
    MSP430_INIT_P65 & MSP430_INIT_PIN_DIRBIT ? 1 : 0) << 5)) | ((
    MSP430_INIT_P66 & MSP430_INIT_PIN_DIRBIT ? 1 : 0) << 6)) | ((
    MSP430_INIT_P67 & MSP430_INIT_PIN_DIRBIT ? 1 : 0) << 7);

    P6OUT = ((((((((MSP430_INIT_P60 & MSP430_INIT_PIN_OUTBIT ? 1 : 0) << 0) | ((
    MSP430_INIT_P61 & MSP430_INIT_PIN_OUTBIT ? 1 : 0) << 1)) | ((
    MSP430_INIT_P62 & MSP430_INIT_PIN_OUTBIT ? 1 : 0) << 2)) | ((
    MSP430_INIT_P63 & MSP430_INIT_PIN_OUTBIT ? 1 : 0) << 3)) | ((
    MSP430_INIT_P64 & MSP430_INIT_PIN_OUTBIT ? 1 : 0) << 4)) | ((
    MSP430_INIT_P65 & MSP430_INIT_PIN_OUTBIT ? 1 : 0) << 5)) | ((
    MSP430_INIT_P66 & MSP430_INIT_PIN_OUTBIT ? 1 : 0) << 6)) | ((
    MSP430_INIT_P67 & MSP430_INIT_PIN_OUTBIT ? 1 : 0) << 7);


    P1IE = 0;
    P2IE = 0;


    TOSH_uwait(1024 * 10);

    TOSH_FLASH_M25P_DP();
  }
}

# 35 "/Users/jingyuancheng/tinyos/moteiv/tinyos-1.x/tos/platform/msp430/HPLInitM.nc"
static inline result_t HPLInitM$init(void )
{
  TOSH_SET_PIN_DIRECTIONS();
  HPLInitM$MSP430ClockControl$init();
  HPLInitM$MSP430ClockControl$start();
  return SUCCESS;
}

# 8 "/Users/jingyuancheng/tinyos/moteiv/tos/platform/msp430/PlatformP.nc"
inline static result_t PlatformP$hplInit(void ){
#line 8
  unsigned char __nesc_result;
#line 8

#line 8
  __nesc_result = HPLInitM$init();
#line 8

#line 8
  return __nesc_result;
#line 8
}
#line 8




static inline result_t PlatformP$HPLInit$init(void )
#line 12
{
  PlatformP$hplInit();
  PlatformP$ArbiterInits$init();
  return SUCCESS;
}

# 46 "/Users/jingyuancheng/tinyos/moteiv/tos/lib/sched/Init.nc"
inline static result_t MainP$PlatformInit$init(void ){
#line 46
  unsigned char __nesc_result;
#line 46

#line 46
  __nesc_result = PlatformP$HPLInit$init();
#line 46

#line 46
  return __nesc_result;
#line 46
}
#line 46
# 19 "/Users/jingyuancheng/tinyos/moteiv/tos/lib/sched/MainP.nc"
static inline void MainP$init(void )
#line 19
{
  uint8_t i;

#line 21
  MainP$PlatformInit$init();
  for (i = 0; i < 255; i++) 
    MainP$MainSplitControl$init(i);
  MainP$StdControl$init();
}

# 38 "/Users/jingyuancheng/tinyos/moteiv/tos/platform/tmote/hardware.h"
static inline void TOSH_CLR_SIMO0_PIN()
#line 38
{
#line 38
  static volatile uint8_t r __asm ("0x0019");

#line 38
  r &= ~(1 << 1);
}

# 50 "/Users/jingyuancheng/tinyos/moteiv/tos/lib/sched/Scheduler.nc"
inline static bool MainP$Scheduler$runNextTask(bool l_sleep){
#line 50
  unsigned char __nesc_result;
#line 50

#line 50
  __nesc_result = SchedulerBasicP$Scheduler$runNextTask(l_sleep);
#line 50

#line 50
  return __nesc_result;
#line 50
}
#line 50
# 103 "/Users/jingyuancheng/tinyos/moteiv/tos/lib/sched/SchedulerBasicP.nc"
static inline uint8_t SchedulerBasicP$popTask(void )
#line 103
{
  uint8_t id = SchedulerBasicP$m_head;

#line 105
  if (id != SchedulerBasicP$NONE) {
      SchedulerBasicP$m_head = SchedulerBasicP$m_next[id];
      if (SchedulerBasicP$m_head == SchedulerBasicP$NONE) {
        SchedulerBasicP$m_tail = SchedulerBasicP$NONE;
        }
#line 109
      SchedulerBasicP$m_next[id] = SchedulerBasicP$NONE;
    }
  return id;
}

# 202 "/Users/jingyuancheng/tinyos/moteiv/tos/platform/msp430/msp430hardware.h"
static inline void __nesc_enable_interrupt()
{
   __asm volatile ("eint");}

#line 245
static __inline void __nesc_atomic_sleep()
#line 245
{








  uint16_t LPMode_bits = 0;

  if (LPMode_disabled) {
      __nesc_enable_interrupt();
      return;
    }
  else 
#line 259
    {
      LPMode_bits = 0x0080 + 0x0040 + 0x0010;
#line 279
      if (((((((
#line 265
      ME1 & ((1 << 7) | (1 << 6)) && U0TCTL & 0x20)

       || (ME2 & ((1 << 5) | (1 << 4)) && U1TCTL & 0x20))



       || (U0CTLnr & 0x01 && I2CTCTLnr & 0x20 && 
      I2CDCTLnr & 0x20 && U0CTLnr & 0x04 && U0CTLnr & 0x20))


       || ((TA0CTL & (3 << 4)) != 0 && (TA0CTL & (3 << 8)) == (2 << 8)))

       || (DMA0CTL & 0x0010 && DMACTL0 & (1 << 0) && (TA0CTL & (3 << 8)) == (2 << 8)))
       || (DMA1CTL & 0x0010 && DMACTL0 & (1 << 4) && (TA0CTL & (3 << 8)) == (2 << 8)))
       || (DMA2CTL & 0x0010 && DMACTL0 & (1 << 8) && (TA0CTL & (3 << 8)) == (2 << 8))) {

        LPMode_bits = 0x0040 + 0x0010;
        }


      if (ADC12CTL1 & 0x0001) {
          if (!(ADC12CTL0 & 0x0080) && (TA0CTL & (3 << 8)) == (2 << 8)) {
            LPMode_bits = 0x0040 + 0x0010;
            }
          else {
#line 289
            switch (ADC12CTL1 & (3 << 3)) {
                case 2 << 3: LPMode_bits = 0;
#line 290
                break;
                case 3 << 3: LPMode_bits = 0x0040 + 0x0010;
#line 291
                break;
              }
            }
        }
      LPMode_bits |= 0x0008;
       __asm volatile ("bis  %0, r2" :  : "m"((uint16_t )LPMode_bits));}
}

# 230 "/Users/jingyuancheng/tinyos/moteiv/tos/lib/resource/FcfsArbiterP.nc"
static inline void /*MSP430ArbiterTimerAC.ArbiterC.ArbiterP*/FcfsArbiterP$0$Resource$default$granted(uint8_t id)
#line 230
{
}

# 73 "/Users/jingyuancheng/tinyos/moteiv/tos/lib/resource/Resource.nc"
inline static void /*MSP430ArbiterTimerAC.ArbiterC.ArbiterP*/FcfsArbiterP$0$Resource$granted(uint8_t arg_0x101b3c648){
#line 73
    /*MSP430ArbiterTimerAC.ArbiterC.ArbiterP*/FcfsArbiterP$0$Resource$default$granted(arg_0x101b3c648);
#line 73
}
#line 73
# 159 "/Users/jingyuancheng/tinyos/moteiv/tos/lib/resource/FcfsArbiterP.nc"
static inline void /*MSP430ArbiterTimerAC.ArbiterC.ArbiterP*/FcfsArbiterP$0$ResourceCmd$release(uint8_t id)
#line 159
{
  /*MSP430ArbiterTimerAC.ArbiterC.ArbiterP*/FcfsArbiterP$0$Resource$release(id);
}

# 75 "/Users/jingyuancheng/tinyos/moteiv/tos/lib/resource/ResourceCmd.nc"
inline static void MSP430DCOCalibP$ResourceTimerA$release(void ){
#line 75
  /*MSP430ArbiterTimerAC.ArbiterC.ArbiterP*/FcfsArbiterP$0$ResourceCmd$release(/*MSP430DCOCalibC.MSP430ResourceTimerAC*/MSP430ResourceTimerAC$0$ID);
#line 75
}
#line 75
# 50 "/Users/jingyuancheng/tinyos/moteiv/tos/platform/msp430/timer/MSP430TimerM.nc"
static inline uint16_t /*MSP430TimerC.MSP430TimerA*/MSP430TimerM$0$Timer$get(void )
{




  if (0) {
      /* atomic removed: atomic calls only */
#line 57
      {
        uint16_t t0;
        uint16_t t1 = * (volatile uint16_t *)368U;

#line 60
        do {
#line 60
            t0 = t1;
#line 60
            t1 = * (volatile uint16_t *)368U;
          }
        while (
#line 60
        t0 != t1);
        {
          unsigned int __nesc_temp = 
#line 61
          t0;

#line 61
          return __nesc_temp;
        }
      }
    }
  else 
#line 64
    {
      return * (volatile uint16_t *)368U;
    }
}

# 30 "/Users/jingyuancheng/tinyos/moteiv/tos/platform/msp430/timer/MSP430Timer.nc"
inline static uint16_t MSP430DCOCalibP$TimerA$get(void ){
#line 30
  unsigned int __nesc_result;
#line 30

#line 30
  __nesc_result = /*MSP430TimerC.MSP430TimerA*/MSP430TimerM$0$Timer$get();
#line 30

#line 30
  return __nesc_result;
#line 30
}
#line 30
# 78 "/Users/jingyuancheng/tinyos/moteiv/tos/platform/msp430/timer/MSP430TimerCapComM.nc"
static inline bool /*MSP430TimerC.MSP430TimerB0*/MSP430TimerCapComM$3$Control$isInterruptPending(void )
{
  return * (volatile uint16_t *)386U & 0x0001;
}

# 31 "/Users/jingyuancheng/tinyos/moteiv/tos/platform/msp430/timer/MSP430TimerControl.nc"
inline static bool MSP430DCOCalibP$TimerControlB$isInterruptPending(void ){
#line 31
  unsigned char __nesc_result;
#line 31

#line 31
  __nesc_result = /*MSP430TimerC.MSP430TimerB0*/MSP430TimerCapComM$3$Control$isInterruptPending();
#line 31

#line 31
  return __nesc_result;
#line 31
}
#line 31
# 83 "/Users/jingyuancheng/tinyos/moteiv/tos/platform/msp430/timer/MSP430TimerCapComM.nc"
static inline void /*MSP430TimerC.MSP430TimerB0*/MSP430TimerCapComM$3$Control$clearPendingInterrupt(void )
{
  * (volatile uint16_t *)386U &= ~0x0001;
}

# 32 "/Users/jingyuancheng/tinyos/moteiv/tos/platform/msp430/timer/MSP430TimerControl.nc"
inline static void MSP430DCOCalibP$TimerControlB$clearPendingInterrupt(void ){
#line 32
  /*MSP430TimerC.MSP430TimerB0*/MSP430TimerCapComM$3$Control$clearPendingInterrupt();
#line 32
}
#line 32
# 148 "/Users/jingyuancheng/tinyos/moteiv/tos/platform/msp430/timer/MSP430TimerCapComM.nc"
static inline void /*MSP430TimerC.MSP430TimerB0*/MSP430TimerCapComM$3$Compare$setEventFromPrev(uint16_t x)
{
  * (volatile uint16_t *)402U += x;
}

# 31 "/Users/jingyuancheng/tinyos/moteiv/tos/platform/msp430/timer/MSP430Compare.nc"
inline static void MSP430DCOCalibP$TimerCompareB$setEventFromPrev(uint16_t delta){
#line 31
  /*MSP430TimerC.MSP430TimerB0*/MSP430TimerCapComM$3$Compare$setEventFromPrev(delta);
#line 31
}
#line 31
# 30 "/Users/jingyuancheng/tinyos/moteiv/tos/platform/msp430/timer/MSP430Timer.nc"
inline static uint16_t /*MSP430TimerC.MSP430TimerB0*/MSP430TimerCapComM$3$Timer$get(void ){
#line 30
  unsigned int __nesc_result;
#line 30

#line 30
  __nesc_result = /*MSP430TimerC.MSP430TimerB*/MSP430TimerM$1$Timer$get();
#line 30

#line 30
  return __nesc_result;
#line 30
}
#line 30
# 153 "/Users/jingyuancheng/tinyos/moteiv/tos/platform/msp430/timer/MSP430TimerCapComM.nc"
static inline void /*MSP430TimerC.MSP430TimerB0*/MSP430TimerCapComM$3$Compare$setEventFromNow(uint16_t x)
{
  * (volatile uint16_t *)402U = /*MSP430TimerC.MSP430TimerB0*/MSP430TimerCapComM$3$Timer$get() + x;
}

# 32 "/Users/jingyuancheng/tinyos/moteiv/tos/platform/msp430/timer/MSP430Compare.nc"
inline static void MSP430DCOCalibP$TimerCompareB$setEventFromNow(uint16_t delta){
#line 32
  /*MSP430TimerC.MSP430TimerB0*/MSP430TimerCapComM$3$Compare$setEventFromNow(delta);
#line 32
}
#line 32
# 123 "/Users/jingyuancheng/tinyos/moteiv/tos/platform/msp430/timer/MSP430TimerCapComM.nc"
static inline void /*MSP430TimerC.MSP430TimerB0*/MSP430TimerCapComM$3$Control$disableEvents(void )
{
  * (volatile uint16_t *)386U &= ~0x0010;
}

# 39 "/Users/jingyuancheng/tinyos/moteiv/tos/platform/msp430/timer/MSP430TimerControl.nc"
inline static void MSP430DCOCalibP$TimerControlB$disableEvents(void ){
#line 39
  /*MSP430TimerC.MSP430TimerB0*/MSP430TimerCapComM$3$Control$disableEvents();
#line 39
}
#line 39
# 43 "/Users/jingyuancheng/tinyos/moteiv/tos/platform/msp430/timer/MSP430DCOCalibP.nc"
static inline uint16_t MSP430DCOCalibP$get_delta_dco(void )
#line 43
{
  uint16_t t0_dco;
  uint16_t t1_dco;

  MSP430DCOCalibP$TimerControlB$disableEvents();

  MSP430DCOCalibP$TimerCompareB$setEventFromNow(2);
  MSP430DCOCalibP$TimerControlB$clearPendingInterrupt();
  while (!MSP430DCOCalibP$TimerControlB$isInterruptPending()) ;
  t0_dco = MSP430DCOCalibP$TimerA$get();

  MSP430DCOCalibP$TimerCompareB$setEventFromPrev(MSP430DCOCalibP$MEASURE_DELTA_RTC);
  MSP430DCOCalibP$TimerControlB$clearPendingInterrupt();
  while (!MSP430DCOCalibP$TimerControlB$isInterruptPending()) ;
  t1_dco = MSP430DCOCalibP$TimerA$get();

  return t1_dco - t0_dco;
}


static inline void MSP430DCOCalibP$step_dco(uint16_t td_dco)
#line 63
{
  if (td_dco > MSP430DCOCalibP$TARGET_DELTA_SMCLK + MSP430DCOCalibP$MAX_SMCLK_DEVIATION) {

      if (DCOCTL > 0) {
          DCOCTL--;
        }
      else {
#line 69
        if ((BCSCTL1 & 7) > 0) {

            BCSCTL1--;
            DCOCTL = 128;
          }
        }
    }
  else {
#line 75
    if (td_dco < MSP430DCOCalibP$TARGET_DELTA_SMCLK - MSP430DCOCalibP$MAX_SMCLK_DEVIATION) {

        if (DCOCTL < 0xe0) {
            DCOCTL++;
          }
        else {
#line 80
          if ((BCSCTL1 & 7) < 7) {

              BCSCTL1++;
              DCOCTL = 96;
            }
          }
      }
    }
}

# 35 "/Users/jingyuancheng/tinyos/moteiv/tos/platform/msp430/timer/MSP430Timer.nc"
inline static void MSP430DCOCalibP$TimerA$setMode(int mode){
#line 35
  /*MSP430TimerC.MSP430TimerA*/MSP430TimerM$0$Timer$setMode(mode);
#line 35
}
#line 35






inline static void MSP430DCOCalibP$TimerA$setInputDivider(uint16_t inputDivider){
#line 41
  /*MSP430TimerC.MSP430TimerA*/MSP430TimerM$0$Timer$setInputDivider(inputDivider);
#line 41
}
#line 41
#line 40
inline static void MSP430DCOCalibP$TimerA$setClockSource(uint16_t clockSource){
#line 40
  /*MSP430TimerC.MSP430TimerA*/MSP430TimerM$0$Timer$setClockSource(clockSource);
#line 40
}
#line 40
# 99 "/Users/jingyuancheng/tinyos/moteiv/tos/platform/msp430/timer/MSP430TimerM.nc"
static inline void /*MSP430TimerC.MSP430TimerA*/MSP430TimerM$0$Timer$disableEvents(void )
{
  * (volatile uint16_t *)352U &= ~2U;
}

# 39 "/Users/jingyuancheng/tinyos/moteiv/tos/platform/msp430/timer/MSP430Timer.nc"
inline static void MSP430DCOCalibP$TimerA$disableEvents(void ){
#line 39
  /*MSP430TimerC.MSP430TimerA*/MSP430TimerM$0$Timer$disableEvents();
#line 39
}
#line 39
# 89 "/Users/jingyuancheng/tinyos/moteiv/tos/platform/msp430/timer/MSP430DCOCalibP.nc"
static inline void MSP430DCOCalibP$ResourceTimerA$granted(uint8_t rh)
#line 89
{
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 90
    {
      MSP430DCOCalibP$TimerA$disableEvents();
      MSP430DCOCalibP$TimerA$setClockSource(MSP430TIMER_SMCLK);
      MSP430DCOCalibP$TimerA$setInputDivider(MSP430TIMER_CLOCKDIV_1);
      MSP430DCOCalibP$TimerA$setMode(MSP430TIMER_CONTINUOUS_MODE);

      MSP430DCOCalibP$step_dco(MSP430DCOCalibP$get_delta_dco());
    }
#line 97
    __nesc_atomic_end(__nesc_atomic); }

  MSP430DCOCalibP$ResourceTimerA$release();
}

# 233 "/Users/jingyuancheng/tinyos/moteiv/tos/lib/resource/FcfsArbiterP.nc"
static inline void /*MSP430ArbiterTimerAC.ArbiterC.ArbiterP*/FcfsArbiterP$0$ResourceCmd$default$granted(uint8_t id, uint8_t rh)
#line 233
{
}

# 70 "/Users/jingyuancheng/tinyos/moteiv/tos/lib/resource/ResourceCmd.nc"
inline static void /*MSP430ArbiterTimerAC.ArbiterC.ArbiterP*/FcfsArbiterP$0$ResourceCmd$granted(uint8_t arg_0x101b3b660, uint8_t rh){
#line 70
  switch (arg_0x101b3b660) {
#line 70
    case /*MSP430DCOCalibC.MSP430ResourceTimerAC*/MSP430ResourceTimerAC$0$ID:
#line 70
      MSP430DCOCalibP$ResourceTimerA$granted(rh);
#line 70
      break;
#line 70
    default:
#line 70
      /*MSP430ArbiterTimerAC.ArbiterC.ArbiterP*/FcfsArbiterP$0$ResourceCmd$default$granted(arg_0x101b3b660, rh);
#line 70
      break;
#line 70
    }
#line 70
}
#line 70
# 531 "/Users/jingyuancheng/tinyos/moteiv/tos/platform/msp430/adc/MSP430ADC12M.nc"
static inline void MSP430ADC12M$Resource$granted(uint8_t _rh)
#line 531
{
}

# 236 "/Users/jingyuancheng/tinyos/moteiv/tos/lib/resource/FcfsArbiterP.nc"
static inline void /*MSP430ArbiterTimerAC.ArbiterC.ArbiterP*/FcfsArbiterP$0$ResourceCmdAsync$default$granted(uint8_t id, uint8_t rh)
#line 236
{
}

# 80 "/Users/jingyuancheng/tinyos/moteiv/tos/lib/resource/ResourceCmdAsync.nc"
inline static void /*MSP430ArbiterTimerAC.ArbiterC.ArbiterP*/FcfsArbiterP$0$ResourceCmdAsync$granted(uint8_t arg_0x101b3a698, uint8_t rh){
#line 80
  switch (arg_0x101b3a698) {
#line 80
    case /*MSP430ADC12C.ResourceC*/MSP430ResourceTimerAC$1$ID:
#line 80
      MSP430ADC12M$Resource$granted(rh);
#line 80
      break;
#line 80
    default:
#line 80
      /*MSP430ArbiterTimerAC.ArbiterC.ArbiterP*/FcfsArbiterP$0$ResourceCmdAsync$default$granted(arg_0x101b3a698, rh);
#line 80
      break;
#line 80
    }
#line 80
}
#line 80
# 40 "/Users/jingyuancheng/tinyos/moteiv/tos/platform/msp430/resource/MSP430ResourceConfigTimerAP.nc"
static inline void MSP430ResourceConfigTimerAP$ConfigTimerA$default$configure(uint8_t rh)
#line 40
{
}

# 29 "/Users/jingyuancheng/tinyos/moteiv/tos/lib/resource/ResourceConfigure.nc"
inline static void MSP430ResourceConfigTimerAP$ConfigTimerA$configure(uint8_t arg_0x101bbdc98){
#line 29
    MSP430ResourceConfigTimerAP$ConfigTimerA$default$configure(arg_0x101bbdc98);
#line 29
}
#line 29
# 34 "/Users/jingyuancheng/tinyos/moteiv/tos/platform/msp430/resource/MSP430ResourceConfigTimerAP.nc"
static inline void MSP430ResourceConfigTimerAP$WrapConfigTimerA$configure(uint8_t rh)
#line 34
{

  MSP430ResourceConfigTimerAP$idle();
  MSP430ResourceConfigTimerAP$ConfigTimerA$configure(rh);
}

# 29 "/Users/jingyuancheng/tinyos/moteiv/tos/lib/resource/ResourceConfigure.nc"
inline static void /*MSP430ArbiterTimerAC.ArbiterC.ArbiterP*/FcfsArbiterP$0$ResourceConfigure$configure(uint8_t arg_0x101b719b0){
#line 29
  MSP430ResourceConfigTimerAP$WrapConfigTimerA$configure(arg_0x101b719b0);
#line 29
}
#line 29
# 57 "/Users/jingyuancheng/tinyos/moteiv/tos/lib/resource/FcfsArbiterP.nc"
static inline void /*MSP430ArbiterTimerAC.ArbiterC.ArbiterP*/FcfsArbiterP$0$GrantTask$runTask(void )
#line 57
{
  uint8_t id;

#line 59
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 59
    id = /*MSP430ArbiterTimerAC.ArbiterC.ArbiterP*/FcfsArbiterP$0$m_granted;
#line 59
    __nesc_atomic_end(__nesc_atomic); }
  if (id != RESOURCE_NONE) {
      { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 61
        /*MSP430ArbiterTimerAC.ArbiterC.ArbiterP*/FcfsArbiterP$0$ResourceConfigure$configure(id);
#line 61
        __nesc_atomic_end(__nesc_atomic); }
      /*MSP430ArbiterTimerAC.ArbiterC.ArbiterP*/FcfsArbiterP$0$ResourceCmdAsync$granted(id, id);
      /*MSP430ArbiterTimerAC.ArbiterC.ArbiterP*/FcfsArbiterP$0$ResourceCmd$granted(id, id);
      /*MSP430ArbiterTimerAC.ArbiterC.ArbiterP*/FcfsArbiterP$0$Resource$granted(id);
    }
}

# 146 "/Users/jingyuancheng/tinyos/moteiv/tos/lib/sp/SPM.nc"
static inline void SPM$PoolEvents$inserted(sp_message_t *msg)
#line 146
{
}

# 20 "/Users/jingyuancheng/tinyos/moteiv/tos/lib/util/pool/ObjectPoolEvents.nc"
inline static void /*SPC.MessagePool*/ObjectPoolC$0$PoolEvents$inserted(/*SPC.MessagePool*/ObjectPoolC$0$PoolEvents$object_type *object){
#line 20
  SPM$PoolEvents$inserted(object);
#line 20
}
#line 20
# 32 "/Users/jingyuancheng/tinyos/moteiv/tos/lib/util/pool/ObjectPoolC.nc"
static inline result_t /*SPC.MessagePool*/ObjectPoolC$0$Pool$insert(/*SPC.MessagePool*/ObjectPoolC$0$object_type *obj)
#line 32
{
  /*SPC.MessagePool*/ObjectPoolC$0$object_type **p;
  /*SPC.MessagePool*/ObjectPoolC$0$object_type **pend = &/*SPC.MessagePool*/ObjectPoolC$0$m_pool[10];

#line 35
  for (p = /*SPC.MessagePool*/ObjectPoolC$0$m_pool + 0; p != pend; p++) {
      if (*p == obj) {
          /*SPC.MessagePool*/ObjectPoolC$0$PoolEvents$inserted(obj);
          return SUCCESS;
        }
    }
  for (p = /*SPC.MessagePool*/ObjectPoolC$0$m_pool + 0; p != pend; p++) {
      if (*p == NULL) {
          *p = obj;
          /*SPC.MessagePool*/ObjectPoolC$0$PoolEvents$inserted(obj);
          return SUCCESS;
        }
    }
  return FAIL;
}

# 24 "/Users/jingyuancheng/tinyos/moteiv/tos/lib/util/pool/ObjectPool.nc"
inline static result_t SPDataM$Pool$insert(SPDataM$Pool$object_type *obj){
#line 24
  unsigned char __nesc_result;
#line 24

#line 24
  __nesc_result = /*SPC.MessagePool*/ObjectPoolC$0$Pool$insert(obj);
#line 24

#line 24
  return __nesc_result;
#line 24
}
#line 24
# 58 "/Users/jingyuancheng/tinyos/moteiv/tinyos-1.x/tos/interfaces/BareSendMsg.nc"
inline static result_t SPDataM$UARTSend$send(TOS_MsgPtr msg){
#line 58
  unsigned char __nesc_result;
#line 58

#line 58
  __nesc_result = FramerP$BareSendMsg$send(msg);
#line 58

#line 58
  return __nesc_result;
#line 58
}
#line 58
# 49 "/Users/jingyuancheng/tinyos/moteiv/tos/lib/timer/LocalTime.nc"
inline static uint32_t SPDataM$LocalTime$get(void ){
#line 49
  unsigned long __nesc_result;
#line 49

#line 49
  __nesc_result = /*Counter32khzC.CounterToLocalTimeC*/CounterToLocalTimeC$1$LocalTime$get();
#line 49

#line 49
  return __nesc_result;
#line 49
}
#line 49
# 31 "/Users/jingyuancheng/tinyos/moteiv/tos/lib/sp/SPDataM.nc"
static inline void SPDataM$setFields(sp_message_t *msg, TOS_Msg *tosmsg, sp_device_t dev, sp_address_t addr, uint8_t length, sp_message_flags_t flags, uint8_t quantity, uint8_t id)
#line 31
{

  msg->msg = tosmsg;


  msg->msg->addr = addr;

  msg->msg->type = id;

  msg->msg->group = TOS_AM_GROUP;

  msg->msg->length = length;


  msg->addr = addr;

  msg->dev = dev == SP_I_NOT_SPECIFIED ? SP_I_RADIO : dev;
  msg->dev = msg->addr == TOS_UART_ADDR ? SP_I_UART : msg->dev;

  msg->id = id;

  msg->flags = flags & ~SP_FLAG_F_ALL;

  msg->quantity = quantity;

  msg->retries = 0;

  msg->length = length;
}






static inline result_t SPDataM$SPSend$sendAdv(uint8_t id, sp_message_t *msg, TOS_Msg *tosmsg, sp_device_t dev, sp_address_t addr, uint8_t length, sp_message_flags_t flags, uint8_t quantity)
#line 66
{


  if (((
#line 68
  tosmsg == NULL || length > 66) || 
  length == 0) || quantity == 0) {
    return FAIL;
    }
  SPDataM$setFields(msg, tosmsg, dev, addr, length, flags, quantity, id);


  msg->time = SPDataM$LocalTime$get();

  if (msg->addr == TOS_UART_ADDR || msg->dev == SP_I_UART) {
      if (SPDataM$m_uartmsg == NULL) {
          if (SPDataM$UARTSend$send(msg->msg) == SUCCESS) {
              SPDataM$m_uartmsg = msg;
              return SUCCESS;
            }
        }
      return FAIL;
    }


  return SPDataM$Pool$insert(msg);
}

# 90 "/Users/jingyuancheng/tinyos/moteiv/tos/lib/sp/SPSend.nc"
inline static result_t SPM$SPDataMgr$sendAdv(uint8_t arg_0x102c5a258, sp_message_t *msg, TOS_Msg *tosmsg, sp_device_t dev, sp_address_t addr, uint8_t length, sp_message_flags_t flags, uint8_t quantity){
#line 90
  unsigned char __nesc_result;
#line 90

#line 90
  __nesc_result = SPDataM$SPSend$sendAdv(arg_0x102c5a258, msg, tosmsg, dev, addr, length, flags, quantity);
#line 90

#line 90
  return __nesc_result;
#line 90
}
#line 90
# 162 "/Users/jingyuancheng/tinyos/moteiv/tos/lib/sp/SPM.nc"
static inline result_t SPM$SPSend$sendAdv(uint8_t id, sp_message_t *_msg, TOS_Msg *_tosmsg, sp_device_t _dev, sp_address_t _addr, uint8_t _length, sp_message_flags_t _flags, uint8_t _quantity)
#line 162
{






  if (
#line 163
  SPM$SPDataMgr$sendAdv(id, _msg, 
  _tosmsg, 
  _dev, 
  _addr, 
  _length, 
  _flags, 
  _quantity) == SUCCESS) {
      SPM$nextSend();
      return SUCCESS;
    }
  return FAIL;
}

#line 152
static inline result_t SPM$SPSend$send(uint8_t id, sp_message_t *_msg, TOS_Msg *_tosmsg, sp_address_t _addr, uint8_t _length)
#line 152
{
  return SPM$SPSend$sendAdv(id, _msg, 
  _tosmsg, 
  _addr == TOS_UART_ADDR ? SP_I_UART : SP_I_RADIO, 
  _addr, 
  _length, 
  SP_FLAG_C_NONE, 
  1);
}

# 46 "/Users/jingyuancheng/tinyos/moteiv/tos/lib/sp/SPSend.nc"
inline static result_t SPAdaptorGenericCommM$SPSend$send(uint8_t arg_0x102d9f488, sp_message_t *msg, TOS_Msg *tosmsg, sp_address_t addr, uint8_t length){
#line 46
  unsigned char __nesc_result;
#line 46

#line 46
  __nesc_result = SPM$SPSend$send(arg_0x102d9f488, msg, tosmsg, addr, length);
#line 46

#line 46
  return __nesc_result;
#line 46
}
#line 46
# 44 "/Users/jingyuancheng/tinyos/moteiv/tos/lib/sp/SPAdaptorGenericCommM.nc"
static inline result_t SPAdaptorGenericCommM$SendMsg$send(uint8_t id, uint16_t addr, uint8_t length, TOS_MsgPtr _msg)
#line 44
{
  sp_message_t *p = SPAdaptorGenericCommM$m_pool + 0;
  const sp_message_t *pend = SPAdaptorGenericCommM$m_pool + 1;

  for (; p != pend; p++) {
      if (p->msg == NULL) {
          p->msg = _msg;


          if (SPAdaptorGenericCommM$SPSend$send(id, p, _msg, addr, length) == SUCCESS) {
            return SUCCESS;
            }
          p->msg = NULL;
          return FAIL;
        }
    }


  return FAIL;
}

# 48 "/Users/jingyuancheng/tinyos/moteiv/tinyos-1.x/tos/interfaces/SendMsg.nc"
inline static result_t ADC8IO14P$SendMsg$send(uint16_t address, uint8_t length, TOS_MsgPtr msg){
#line 48
  unsigned char __nesc_result;
#line 48

#line 48
  __nesc_result = SPAdaptorGenericCommM$SendMsg$send(31, address, length, msg);
#line 48

#line 48
  return __nesc_result;
#line 48
}
#line 48
# 219 "ADC8IO14P.nc"
static inline void ADC8IO14P$taskSendData$runTask(void )
{

  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
    {
      if (ADC8IO14P$SendMsg$send(TOS_UART_ADDR, sizeof(ADCMsg_t ), &ADC8IO14P$m_msg) == SUCCESS) 
        {
          ADC8IO14P$m_sending = TRUE;
        }
    }
#line 228
    __nesc_atomic_end(__nesc_atomic); }
}

# 91 "/Users/jingyuancheng/tinyos/moteiv/tos/lib/timer/AlarmToTimerC.nc"
static inline uint32_t /*HalTimerMilliC.AlarmToTimerC*/AlarmToTimerC$0$Timer$getdt(void )
#line 91
{
  return /*HalTimerMilliC.AlarmToTimerC*/AlarmToTimerC$0$m_dt;
}

# 129 "/Users/jingyuancheng/tinyos/moteiv/tos/lib/timer/Timer2.nc"
inline static uint32_t /*HalTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC$0$TimerFrom$getdt(void ){
#line 129
  unsigned long __nesc_result;
#line 129

#line 129
  __nesc_result = /*HalTimerMilliC.AlarmToTimerC*/AlarmToTimerC$0$Timer$getdt();
#line 129

#line 129
  return __nesc_result;
#line 129
}
#line 129
# 87 "/Users/jingyuancheng/tinyos/moteiv/tos/lib/timer/AlarmToTimerC.nc"
static inline uint32_t /*HalTimerMilliC.AlarmToTimerC*/AlarmToTimerC$0$Timer$gett0(void )
#line 87
{
  return /*HalTimerMilliC.AlarmToTimerC*/AlarmToTimerC$0$m_t0;
}

# 123 "/Users/jingyuancheng/tinyos/moteiv/tos/lib/timer/Timer2.nc"
inline static uint32_t /*HalTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC$0$TimerFrom$gett0(void ){
#line 123
  unsigned long __nesc_result;
#line 123

#line 123
  __nesc_result = /*HalTimerMilliC.AlarmToTimerC*/AlarmToTimerC$0$Timer$gett0();
#line 123

#line 123
  return __nesc_result;
#line 123
}
#line 123
# 142 "/Users/jingyuancheng/tinyos/moteiv/tos/lib/timer/VirtualizeTimerC.nc"
static inline void /*HalTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC$0$TimerFrom$fired(void )
{
  /*HalTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC$0$executeTimers(/*HalTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC$0$TimerFrom$gett0() + /*HalTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC$0$TimerFrom$getdt());
}

# 68 "/Users/jingyuancheng/tinyos/moteiv/tos/lib/timer/Timer2.nc"
inline static void /*HalTimerMilliC.AlarmToTimerC*/AlarmToTimerC$0$Timer$fired(void ){
#line 68
  /*HalTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC$0$TimerFrom$fired();
#line 68
}
#line 68
# 88 "/Users/jingyuancheng/tinyos/moteiv/tos/lib/timer/Alarm.nc"
inline static void /*HalTimerMilliC.AlarmToTimerC*/AlarmToTimerC$0$Alarm$startAt(/*HalTimerMilliC.AlarmToTimerC*/AlarmToTimerC$0$Alarm$size_type t0, /*HalTimerMilliC.AlarmToTimerC*/AlarmToTimerC$0$Alarm$size_type dt){
#line 88
  /*HalTimerMilliC.AlarmMilliC.Transform*/TransformAlarmC$0$Alarm$startAt(t0, dt);
#line 88
}
#line 88
# 38 "/Users/jingyuancheng/tinyos/moteiv/tos/lib/timer/AlarmToTimerC.nc"
static inline void /*HalTimerMilliC.AlarmToTimerC*/AlarmToTimerC$0$start(uint32_t t0, uint32_t dt, bool oneshot)
#line 38
{
  /*HalTimerMilliC.AlarmToTimerC*/AlarmToTimerC$0$m_t0 = t0;
  /*HalTimerMilliC.AlarmToTimerC*/AlarmToTimerC$0$m_dt = dt;
  /*HalTimerMilliC.AlarmToTimerC*/AlarmToTimerC$0$m_oneshot = oneshot;
  /*HalTimerMilliC.AlarmToTimerC*/AlarmToTimerC$0$Alarm$startAt(t0, dt);
}

#line 57
static inline void /*HalTimerMilliC.AlarmToTimerC*/AlarmToTimerC$0$fired$runTask(void )
#line 57
{
  if (/*HalTimerMilliC.AlarmToTimerC*/AlarmToTimerC$0$m_oneshot == FALSE) {
    /*HalTimerMilliC.AlarmToTimerC*/AlarmToTimerC$0$start(/*HalTimerMilliC.AlarmToTimerC*/AlarmToTimerC$0$m_t0 + /*HalTimerMilliC.AlarmToTimerC*/AlarmToTimerC$0$m_dt, /*HalTimerMilliC.AlarmToTimerC*/AlarmToTimerC$0$m_dt, FALSE);
    }
#line 60
  /*HalTimerMilliC.AlarmToTimerC*/AlarmToTimerC$0$Timer$fired();
}

# 52 "/Users/jingyuancheng/tinyos/moteiv/tos/lib/timer/Counter.nc"
inline static /*HalTimerMilliC.AlarmMilliC.Transform*/TransformAlarmC$0$Counter$size_type /*HalTimerMilliC.AlarmMilliC.Transform*/TransformAlarmC$0$Counter$get(void ){
#line 52
  unsigned long __nesc_result;
#line 52

#line 52
  __nesc_result = /*CounterMilliC.Transform*/TransformCounterC$0$Counter$get();
#line 52

#line 52
  return __nesc_result;
#line 52
}
#line 52
# 49 "/Users/jingyuancheng/tinyos/moteiv/tos/lib/timer/TransformAlarmC.nc"
static inline /*HalTimerMilliC.AlarmMilliC.Transform*/TransformAlarmC$0$to_size_type /*HalTimerMilliC.AlarmMilliC.Transform*/TransformAlarmC$0$Alarm$getNow(void )
{
  return /*HalTimerMilliC.AlarmMilliC.Transform*/TransformAlarmC$0$Counter$get();
}

# 93 "/Users/jingyuancheng/tinyos/moteiv/tos/lib/timer/Alarm.nc"
inline static /*HalTimerMilliC.AlarmToTimerC*/AlarmToTimerC$0$Alarm$size_type /*HalTimerMilliC.AlarmToTimerC*/AlarmToTimerC$0$Alarm$getNow(void ){
#line 93
  unsigned long __nesc_result;
#line 93

#line 93
  __nesc_result = /*HalTimerMilliC.AlarmMilliC.Transform*/TransformAlarmC$0$Alarm$getNow();
#line 93

#line 93
  return __nesc_result;
#line 93
}
#line 93
# 83 "/Users/jingyuancheng/tinyos/moteiv/tos/lib/timer/AlarmToTimerC.nc"
static inline uint32_t /*HalTimerMilliC.AlarmToTimerC*/AlarmToTimerC$0$Timer$getNow(void )
#line 83
{
  return /*HalTimerMilliC.AlarmToTimerC*/AlarmToTimerC$0$Alarm$getNow();
}

# 116 "/Users/jingyuancheng/tinyos/moteiv/tos/lib/timer/Timer2.nc"
inline static uint32_t /*HalTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC$0$TimerFrom$getNow(void ){
#line 116
  unsigned long __nesc_result;
#line 116

#line 116
  __nesc_result = /*HalTimerMilliC.AlarmToTimerC*/AlarmToTimerC$0$Timer$getNow();
#line 116

#line 116
  return __nesc_result;
#line 116
}
#line 116
# 56 "/Users/jingyuancheng/tinyos/moteiv/tos/platform/msp430/timer/MSP430AlarmC.nc"
static inline void /*HalTimerMilliC.AlarmMilliC.MSP430Alarm*/MSP430AlarmC$0$Alarm$stop(void )
{
  /*HalTimerMilliC.AlarmMilliC.MSP430Alarm*/MSP430AlarmC$0$MSP430TimerControl$disableEvents();
}

# 60 "/Users/jingyuancheng/tinyos/moteiv/tos/lib/timer/Alarm.nc"
inline static void /*HalTimerMilliC.AlarmMilliC.Transform*/TransformAlarmC$0$AlarmFrom$stop(void ){
#line 60
  /*HalTimerMilliC.AlarmMilliC.MSP430Alarm*/MSP430AlarmC$0$Alarm$stop();
#line 60
}
#line 60
# 65 "/Users/jingyuancheng/tinyos/moteiv/tos/lib/timer/TransformAlarmC.nc"
static inline void /*HalTimerMilliC.AlarmMilliC.Transform*/TransformAlarmC$0$Alarm$stop(void )
{
  /*HalTimerMilliC.AlarmMilliC.Transform*/TransformAlarmC$0$AlarmFrom$stop();
}

# 60 "/Users/jingyuancheng/tinyos/moteiv/tos/lib/timer/Alarm.nc"
inline static void /*HalTimerMilliC.AlarmToTimerC*/AlarmToTimerC$0$Alarm$stop(void ){
#line 60
  /*HalTimerMilliC.AlarmMilliC.Transform*/TransformAlarmC$0$Alarm$stop();
#line 60
}
#line 60
# 53 "/Users/jingyuancheng/tinyos/moteiv/tos/lib/timer/AlarmToTimerC.nc"
static inline void /*HalTimerMilliC.AlarmToTimerC*/AlarmToTimerC$0$Timer$stop(void )
#line 53
{
  /*HalTimerMilliC.AlarmToTimerC*/AlarmToTimerC$0$Alarm$stop();
}

# 64 "/Users/jingyuancheng/tinyos/moteiv/tos/lib/timer/Timer2.nc"
inline static void /*HalTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC$0$TimerFrom$stop(void ){
#line 64
  /*HalTimerMilliC.AlarmToTimerC*/AlarmToTimerC$0$Timer$stop();
#line 64
}
#line 64
# 147 "/Users/jingyuancheng/tinyos/moteiv/tos/lib/timer/VirtualizeTimerC.nc"
static inline void /*HalTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC$0$executeTimersNow$runTask(void )
{
  /*HalTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC$0$TimerFrom$stop();
  /*HalTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC$0$executeTimers(/*HalTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC$0$TimerFrom$getNow());
}

# 126 "/Users/jingyuancheng/tinyos/moteiv/tos/lib/sp/SPSend.nc"
inline static void SPM$SPSend$sendDone(uint8_t arg_0x102c62ca8, sp_message_t *msg, sp_message_flags_t flags, sp_error_t error){
#line 126
  SPAdaptorGenericCommM$SPSend$sendDone(arg_0x102c62ca8, msg, flags, error);
#line 126
}
#line 126
# 214 "/Users/jingyuancheng/tinyos/moteiv/tos/lib/sp/SPM.nc"
static inline void SPM$SPDataMgr$sendDone(uint8_t id, sp_message_t *msg, sp_message_flags_t flags, sp_error_t success)
#line 214
{
  SPM$SPSend$sendDone(id, msg, flags, success);
}

# 126 "/Users/jingyuancheng/tinyos/moteiv/tos/lib/sp/SPSend.nc"
inline static void SPDataM$SPSend$sendDone(uint8_t arg_0x102cbc0c8, sp_message_t *msg, sp_message_flags_t flags, sp_error_t error){
#line 126
  SPM$SPDataMgr$sendDone(arg_0x102cbc0c8, msg, flags, error);
#line 126
}
#line 126
# 381 "/Users/jingyuancheng/tinyos/moteiv/tos/lib/sp/SPM.nc"
static inline void SPM$SPSendNext$default$request(uint8_t id, sp_message_t *msg, TOS_Msg *tosmsg, uint8_t remaining)
#line 381
{
}

# 41 "/Users/jingyuancheng/tinyos/moteiv/tos/lib/sp/SPSendNext.nc"
inline static void SPM$SPSendNext$request(uint8_t arg_0x102c5ee00, sp_message_t *msg, TOS_Msg *tosmsg, uint8_t remaining){
#line 41
    SPM$SPSendNext$default$request(arg_0x102c5ee00, msg, tosmsg, remaining);
#line 41
}
#line 41
# 210 "/Users/jingyuancheng/tinyos/moteiv/tos/lib/sp/SPM.nc"
static inline void SPM$SPDataMgrNext$request(uint8_t id, sp_message_t *msg, TOS_Msg *tosmsg, uint8_t quantity)
#line 210
{
  SPM$SPSendNext$request(id, msg, tosmsg, quantity);
}

# 41 "/Users/jingyuancheng/tinyos/moteiv/tos/lib/sp/SPSendNext.nc"
inline static void SPDataM$SPSendNext$request(uint8_t arg_0x102cba220, sp_message_t *msg, TOS_Msg *tosmsg, uint8_t remaining){
#line 41
  SPM$SPDataMgrNext$request(arg_0x102cba220, msg, tosmsg, remaining);
#line 41
}
#line 41
# 113 "/Users/jingyuancheng/tinyos/moteiv/tos/lib/sp/SPDataM.nc"
static inline result_t SPDataM$UARTSend$sendDone(TOS_MsgPtr msg, result_t success)
#line 113
{
  sp_message_t *_stack = SPDataM$m_uartmsg;

  if (SPDataM$m_uartmsg != NULL && msg == SPDataM$m_uartmsg->msg) {

      SPDataM$m_uartmsg->quantity--;
      if (SPDataM$m_uartmsg->quantity > 0) {
          SPDataM$m_uartmsg->msg = NULL;
          SPDataM$m_uartmsg->flags |= SP_FLAG_C_FUTURES;

          SPDataM$SPSendNext$request(SPDataM$m_uartmsg->id, SPDataM$m_uartmsg, 
          msg, 
          SPDataM$m_uartmsg->quantity);

          SPDataM$m_uartmsg->flags &= ~SP_FLAG_C_FUTURES;
          if (SPDataM$m_uartmsg->msg != NULL) {
              if (SPDataM$UARTSend$send(SPDataM$m_uartmsg->msg) == SUCCESS) {
                  return SUCCESS;
                }
            }
        }

      SPDataM$m_uartmsg = NULL;
      SPDataM$SPSend$sendDone(_stack->id, _stack, _stack->flags, success != SUCCESS);
    }
  return SUCCESS;
}

# 67 "/Users/jingyuancheng/tinyos/moteiv/tinyos-1.x/tos/interfaces/BareSendMsg.nc"
inline static result_t FramerP$BareSendMsg$sendDone(TOS_MsgPtr msg, result_t success){
#line 67
  unsigned char __nesc_result;
#line 67

#line 67
  __nesc_result = SPDataM$UARTSend$sendDone(msg, success);
#line 67

#line 67
  return __nesc_result;
#line 67
}
#line 67
# 52 "/Users/jingyuancheng/tinyos/moteiv/tos/lib/timer/Timer2.nc"
inline static void TimerWrapC$Timer2$startPeriodic(uint8_t arg_0x1025b4258, uint32_t dt){
#line 52
  /*HalTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC$0$Timer$startPeriodic(arg_0x1025b4258, dt);
#line 52
}
#line 52
# 167 "/Users/jingyuancheng/tinyos/moteiv/tos/lib/timer/VirtualizeTimerC.nc"
static inline void /*HalTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC$0$Timer$startOneShot(uint8_t num, uint32_t dt)
{
  /*HalTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC$0$startTimer(num, /*HalTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC$0$TimerFrom$getNow(), dt, TRUE);
}

# 60 "/Users/jingyuancheng/tinyos/moteiv/tos/lib/timer/Timer2.nc"
inline static void TimerWrapC$Timer2$startOneShot(uint8_t arg_0x1025b4258, uint32_t dt){
#line 60
  /*HalTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC$0$Timer$startOneShot(arg_0x1025b4258, dt);
#line 60
}
#line 60
# 24 "/Users/jingyuancheng/tinyos/moteiv/tos/lib/timer/TimerWrapC.nc"
static inline result_t TimerWrapC$Timer$start(uint8_t id, char type, uint32_t interval)
{
  if (type == TIMER_ONE_SHOT) 
    {
      TimerWrapC$Timer2$startOneShot(id, interval);
      return SUCCESS;
    }

  if (type == TIMER_REPEAT) 
    {
      TimerWrapC$Timer2$startPeriodic(id, interval);
      return SUCCESS;
    }

  return FAIL;
}

# 59 "/Users/jingyuancheng/tinyos/moteiv/tinyos-1.x/tos/interfaces/Timer.nc"
inline static result_t FramerP$Timer$start(char type, uint32_t interval){
#line 59
  unsigned char __nesc_result;
#line 59

#line 59
  __nesc_result = TimerWrapC$Timer$start(1U, type, interval);
#line 59

#line 59
  return __nesc_result;
#line 59
}
#line 59
# 254 "/Users/jingyuancheng/tinyos/moteiv/tos/platform/tmote/FramerP.nc"
static inline void FramerP$PacketSent$runTask(void )
#line 254
{
  result_t TxResult = SUCCESS;



  FramerP$Timer$start(TIMER_ONE_SHOT, 5);

  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 261
    {
      if (FramerP$gTxState == FramerP$TXSTATE_ERROR) {
          TxResult = FAIL;
          FramerP$gTxState = FramerP$TXSTATE_IDLE;
        }
    }
#line 266
    __nesc_atomic_end(__nesc_atomic); }
  if (FramerP$gTxProto == FramerP$PROTO_ACK) {
      { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 268
        FramerP$gFlags ^= FramerP$FLAGS_TOKENPEND;
#line 268
        __nesc_atomic_end(__nesc_atomic); }
    }
  else {
      TOS_Msg *_stack;

#line 272
      { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 272
        {
          _stack = (TOS_Msg *)FramerP$gpTxMsg;
          FramerP$gFlags ^= FramerP$FLAGS_DATAPEND;
          FramerP$gpTxMsg = NULL;
        }
#line 276
        __nesc_atomic_end(__nesc_atomic); }
      FramerP$BareSendMsg$sendDone(_stack, TxResult);
    }


  FramerP$StartTx();
}

#line 205
static inline void FramerP$PacketUnknown$runTask(void )
#line 205
{
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 206
    {
      FramerP$gFlags |= FramerP$FLAGS_UNKNOWN;
    }
#line 208
    __nesc_atomic_end(__nesc_atomic); }

  FramerP$StartTx();
}

# 49 "/Users/jingyuancheng/tinyos/moteiv/tos/lib/sched/TaskBasic.nc"
inline static result_t FramerP$PacketUnknown$postTask(void ){
#line 49
  unsigned char __nesc_result;
#line 49

#line 49
  __nesc_result = SchedulerBasicP$TaskBasic$postTask(FramerP$PacketUnknown);
#line 49

#line 49
  return __nesc_result;
#line 49
}
#line 49
# 75 "/Users/jingyuancheng/tinyos/moteiv/tinyos-1.x/tos/interfaces/ReceiveMsg.nc"
inline static TOS_MsgPtr FramerAckM$ReceiveCombined$receive(TOS_MsgPtr m){
#line 75
  struct TOS_Msg *__nesc_result;
#line 75

#line 75
  __nesc_result = SPM$UARTReceive$receive(m);
#line 75

#line 75
  return __nesc_result;
#line 75
}
#line 75
# 91 "/Users/jingyuancheng/tinyos/moteiv/tinyos-1.x/tos/system/FramerAckM.nc"
static inline TOS_MsgPtr FramerAckM$ReceiveMsg$receive(TOS_MsgPtr Msg)
#line 91
{
  TOS_MsgPtr pBuf;

  pBuf = FramerAckM$ReceiveCombined$receive(Msg);

  return pBuf;
}

# 75 "/Users/jingyuancheng/tinyos/moteiv/tinyos-1.x/tos/interfaces/ReceiveMsg.nc"
inline static TOS_MsgPtr FramerP$ReceiveMsg$receive(TOS_MsgPtr m){
#line 75
  struct TOS_Msg *__nesc_result;
#line 75

#line 75
  __nesc_result = FramerAckM$ReceiveMsg$receive(m);
#line 75

#line 75
  return __nesc_result;
#line 75
}
#line 75
# 49 "/Users/jingyuancheng/tinyos/moteiv/tos/lib/sched/TaskBasic.nc"
inline static result_t FramerAckM$SendAckTask$postTask(void ){
#line 49
  unsigned char __nesc_result;
#line 49

#line 49
  __nesc_result = SchedulerBasicP$TaskBasic$postTask(FramerAckM$SendAckTask);
#line 49

#line 49
  return __nesc_result;
#line 49
}
#line 49
# 79 "/Users/jingyuancheng/tinyos/moteiv/tinyos-1.x/tos/system/FramerAckM.nc"
static inline TOS_MsgPtr FramerAckM$TokenReceiveMsg$receive(TOS_MsgPtr Msg, uint8_t token)
#line 79
{
  TOS_MsgPtr pBuf;

  FramerAckM$gTokenBuf = token;

  FramerAckM$SendAckTask$postTask();

  pBuf = FramerAckM$ReceiveCombined$receive(Msg);

  return pBuf;
}

# 75 "/Users/jingyuancheng/tinyos/moteiv/tinyos-1.x/tos/interfaces/TokenReceiveMsg.nc"
inline static TOS_MsgPtr FramerP$TokenReceiveMsg$receive(TOS_MsgPtr Msg, uint8_t Token){
#line 75
  struct TOS_Msg *__nesc_result;
#line 75

#line 75
  __nesc_result = FramerAckM$TokenReceiveMsg$receive(Msg, Token);
#line 75

#line 75
  return __nesc_result;
#line 75
}
#line 75
# 213 "/Users/jingyuancheng/tinyos/moteiv/tos/platform/tmote/FramerP.nc"
static inline void FramerP$PacketRcvd$runTask(void )
#line 213
{
  FramerP$MsgRcvEntry_t *pRcv = &FramerP$gMsgRcvTbl[FramerP$gRxTailIndex];
  TOS_MsgPtr pBuf = pRcv->pMsg;


  if (pRcv->Length >= (size_t )& ((TOS_Msg *)0)->data) {

      switch (pRcv->Proto) {
          case FramerP$PROTO_ACK: 
            break;
          case FramerP$PROTO_PACKET_ACK: 
            pBuf->crc = 1;
          pBuf = FramerP$TokenReceiveMsg$receive(pBuf, pRcv->Token);
          break;
          case FramerP$PROTO_PACKET_NOACK: 
            pBuf->crc = 1;
          pBuf = FramerP$ReceiveMsg$receive(pBuf);
          break;
          default: 
            FramerP$gTxUnknownBuf = pRcv->Proto;
          FramerP$PacketUnknown$postTask();
          break;
        }
    }

  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 238
    {
      if (pBuf) {
          pRcv->pMsg = pBuf;
        }
      pRcv->Length = 0;
      pRcv->Token = 0;
      FramerP$gRxTailIndex++;
      FramerP$gRxTailIndex %= FramerP$HDLC_QUEUESIZE;
    }
#line 246
    __nesc_atomic_end(__nesc_atomic); }
}

# 44 "/Users/jingyuancheng/tinyos/moteiv/tos/platform/tmote/hardware.h"
static inline void TOSH_SEL_URXD1_MODFUNC()
#line 44
{
#line 44
  static volatile uint8_t r __asm ("0x001B");

#line 44
  r |= 1 << 7;
}

# 160 "/Users/jingyuancheng/tinyos/moteiv/tos/platform/msp430/HPLUSART1M.nc"
static inline void HPLUSART1M$USARTControl$enableUARTRx(void )
#line 160
{
  TOSH_SEL_URXD1_MODFUNC();
  HPLUSART1M$ME2 |= 1 << 4;
}

# 100 "/Users/jingyuancheng/tinyos/moteiv/tinyos-1.x/tos/platform/msp430/HPLUSARTControl.nc"
inline static void FramerP$USARTControl$enableUARTRx(void ){
#line 100
  HPLUSART1M$USARTControl$enableUARTRx();
#line 100
}
#line 100
# 595 "/Users/jingyuancheng/tinyos/moteiv/tos/platform/tmote/FramerP.nc"
static inline void FramerP$Detect$connected(void )
#line 595
{
  FramerP$USARTControl$enableUARTRx();
}

# 28 "/Users/jingyuancheng/tinyos/moteiv/tos/interfaces/Detect.nc"
inline static void UartPresenceM$Presence$connected(void ){
#line 28
  FramerP$Detect$connected();
#line 28
}
#line 28
# 43 "/Users/jingyuancheng/tinyos/moteiv/tos/platform/tmote/util/uartdetect/UartPresenceM.nc"
static inline void UartPresenceM$taskConnected$runTask(void )
#line 43
{
  UartPresenceM$Presence$connected();
}

# 44 "/Users/jingyuancheng/tinyos/moteiv/tos/platform/tmote/hardware.h"
static inline void TOSH_SEL_URXD1_IOFUNC()
#line 44
{
#line 44
  static volatile uint8_t r __asm ("0x001B");

#line 44
  r &= ~(1 << 7);
}

# 165 "/Users/jingyuancheng/tinyos/moteiv/tos/platform/msp430/HPLUSART1M.nc"
static inline void HPLUSART1M$USARTControl$disableUARTRx(void )
#line 165
{
  HPLUSART1M$ME2 &= ~(1 << 4);
  TOSH_SEL_URXD1_IOFUNC();
}

# 105 "/Users/jingyuancheng/tinyos/moteiv/tinyos-1.x/tos/platform/msp430/HPLUSARTControl.nc"
inline static void FramerP$USARTControl$disableUARTRx(void ){
#line 105
  HPLUSART1M$USARTControl$disableUARTRx();
#line 105
}
#line 105
# 599 "/Users/jingyuancheng/tinyos/moteiv/tos/platform/tmote/FramerP.nc"
static inline void FramerP$Detect$disconnected(void )
#line 599
{
  FramerP$USARTControl$disableUARTRx();
}

# 33 "/Users/jingyuancheng/tinyos/moteiv/tos/interfaces/Detect.nc"
inline static void UartPresenceM$Presence$disconnected(void ){
#line 33
  FramerP$Detect$disconnected();
#line 33
}
#line 33
# 47 "/Users/jingyuancheng/tinyos/moteiv/tos/platform/tmote/util/uartdetect/UartPresenceM.nc"
static inline void UartPresenceM$taskDisconnected$runTask(void )
#line 47
{
  UartPresenceM$Presence$disconnected();
}

# 354 "/Users/jingyuancheng/tinyos/moteiv/tos/platform/tmote/FramerP.nc"
static inline result_t FramerP$TokenReceiveMsg$ReflectToken(uint8_t Token)
#line 354
{
  result_t Result = SUCCESS;

  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 357
    {
      if (!(FramerP$gFlags & FramerP$FLAGS_TOKENPEND)) {
          FramerP$gFlags |= FramerP$FLAGS_TOKENPEND;
          FramerP$gTxTokenBuf = Token;
        }
      else {
          Result = FAIL;
        }
    }
#line 365
    __nesc_atomic_end(__nesc_atomic); }

  if (Result == SUCCESS) {
      Result = FramerP$StartTx();
    }

  return Result;
}

# 88 "/Users/jingyuancheng/tinyos/moteiv/tinyos-1.x/tos/interfaces/TokenReceiveMsg.nc"
inline static result_t FramerAckM$TokenReceiveMsg$ReflectToken(uint8_t Token){
#line 88
  unsigned char __nesc_result;
#line 88

#line 88
  __nesc_result = FramerP$TokenReceiveMsg$ReflectToken(Token);
#line 88

#line 88
  return __nesc_result;
#line 88
}
#line 88
# 74 "/Users/jingyuancheng/tinyos/moteiv/tinyos-1.x/tos/system/FramerAckM.nc"
static inline void FramerAckM$SendAckTask$runTask(void )
#line 74
{

  FramerAckM$TokenReceiveMsg$ReflectToken(FramerAckM$gTokenBuf);
}

# 155 "/Users/jingyuancheng/tinyos/moteiv/tos/lib/CC2420Radio/CC2420RadioM.nc"
static inline void CC2420RadioM$sendFailedTask$runTask(void )
#line 155
{
  CC2420RadioM$sendFailedSync();
}

# 384 "/Users/jingyuancheng/tinyos/moteiv/tos/lib/sp/SPM.nc"
static inline TOS_MsgPtr SPM$ReceiveMsg$default$receive(uint8_t id, TOS_MsgPtr m)
#line 384
{
#line 384
  return m;
}

# 75 "/Users/jingyuancheng/tinyos/moteiv/tinyos-1.x/tos/interfaces/ReceiveMsg.nc"
inline static TOS_MsgPtr SPM$ReceiveMsg$receive(uint8_t arg_0x102c5c618, TOS_MsgPtr m){
#line 75
  struct TOS_Msg *__nesc_result;
#line 75

#line 75
    __nesc_result = SPM$ReceiveMsg$default$receive(arg_0x102c5c618, m);
#line 75

#line 75
  return __nesc_result;
#line 75
}
#line 75
# 383 "/Users/jingyuancheng/tinyos/moteiv/tos/lib/sp/SPM.nc"
static inline void SPM$SPReceive$default$receive(uint8_t id, sp_message_t *spmsg, TOS_MsgPtr m, sp_error_t result)
#line 383
{
}

# 40 "/Users/jingyuancheng/tinyos/moteiv/tos/lib/sp/SPReceive.nc"
inline static void SPM$SPReceive$receive(uint8_t arg_0x102c5dae8, sp_message_t *spmsg, TOS_MsgPtr tosmsg, sp_error_t result){
#line 40
    SPM$SPReceive$default$receive(arg_0x102c5dae8, spmsg, tosmsg, result);
#line 40
}
#line 40
# 351 "/Users/jingyuancheng/tinyos/moteiv/tos/lib/sp/SPM.nc"
static inline TOS_MsgPtr SPM$LowerReceive$receive(TOS_MsgPtr m)
#line 351
{

  if (m->group == TOS_AM_GROUP) {
      sp_message_t spmsg;

#line 355
      SPM$setRxFields(&spmsg, m, SP_I_RADIO, SP_FLAG_C_NONE, 1, m->type);
      SPM$SPReceive$receive(m->type, &spmsg, m, SP_SUCCESS);
      return SPM$ReceiveMsg$receive(m->type, m);
    }
  return m;
}

# 75 "/Users/jingyuancheng/tinyos/moteiv/tinyos-1.x/tos/interfaces/ReceiveMsg.nc"
inline static TOS_MsgPtr CC2420RadioM$Receive$receive(TOS_MsgPtr m){
#line 75
  struct TOS_Msg *__nesc_result;
#line 75

#line 75
  __nesc_result = SPM$LowerReceive$receive(m);
#line 75

#line 75
  return __nesc_result;
#line 75
}
#line 75
# 220 "/Users/jingyuancheng/tinyos/moteiv/tos/lib/CC2420Radio/CC2420RadioM.nc"
static inline void CC2420RadioM$PacketRcvd$runTask(void )
#line 220
{
  TOS_MsgPtr pBuf;

  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 223
    {
      pBuf = CC2420RadioM$rxbufptr;
    }
#line 225
    __nesc_atomic_end(__nesc_atomic); }
  if (CC2420RadioM$rxbufptr->crc) {
    pBuf = CC2420RadioM$Receive$receive((TOS_MsgPtr )pBuf);
    }
#line 228
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 228
    {
      if (pBuf) {
#line 229
        CC2420RadioM$rxbufptr = pBuf;
        }
#line 230
      CC2420RadioM$rxbufptr->length = 0;
      CC2420RadioM$bPacketReceiving = FALSE;
      if (CC2420RadioM$m_rxFifoCount > 0 && --CC2420RadioM$m_rxFifoCount > 0) {
        CC2420RadioM$CmdReceive$deferRequest();
        }
    }
#line 235
    __nesc_atomic_end(__nesc_atomic); }
}

# 68 "/Users/jingyuancheng/tinyos/moteiv/tos/lib/CC2420Radio/CC2420BareSendMsg.nc"
inline static result_t CC2420RadioM$Send$sendDone(TOS_MsgPtr msg, cc2420_error_t success){
#line 68
  unsigned char __nesc_result;
#line 68

#line 68
  __nesc_result = CC2420AlwaysOnM$LowerSend$sendDone(msg, success);
#line 68

#line 68
  return __nesc_result;
#line 68
}
#line 68
# 238 "/Users/jingyuancheng/tinyos/moteiv/tos/lib/CC2420Radio/CC2420RadioM.nc"
static inline void CC2420RadioM$PacketSent$runTask(void )
#line 238
{
  TOS_MsgPtr pBuf;

  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 241
    {
      CC2420RadioM$stateRadio = CC2420RadioM$IDLE_STATE;
      pBuf = CC2420RadioM$txbufptr;
      pBuf->length = pBuf->length - MSG_HEADER_SIZE - MSG_FOOTER_SIZE;
    }
#line 245
    __nesc_atomic_end(__nesc_atomic); }

  CC2420RadioM$Send$sendDone(pBuf, CC2420_SUCCESS);
}

# 63 "/Users/jingyuancheng/tinyos/moteiv/tos/lib/resource/ResourceCmd.nc"
inline static void CC2420ControlM$CmdSplitControlStop$deferRequest(void ){
#line 63
  /*MSP430ArbiterUSART0C.ArbiterC.ArbiterP*/FcfsArbiterP$1$ResourceCmd$deferRequest(/*CC2420RadioC.CmdSplitControlStopC.ResourceC.ResourceUSARTP*/MSP430ResourceUSART0P$3$ID);
#line 63
}
#line 63
# 206 "/Users/jingyuancheng/tinyos/moteiv/tos/lib/CC2420Radio/CC2420ControlM.nc"
static inline result_t CC2420ControlM$SplitControl$stop(void )
#line 206
{
  uint8_t _state = FALSE;

  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 209
    {
      if (CC2420ControlM$state == CC2420ControlM$START_STATE_DONE) {
          CC2420ControlM$state = CC2420ControlM$STOP_STATE;
          _state = TRUE;
        }
    }
#line 214
    __nesc_atomic_end(__nesc_atomic); }
  if (!_state) {
    return FAIL;
    }
  CC2420ControlM$CmdSplitControlStop$deferRequest();
  return SUCCESS;
}

# 93 "/Users/jingyuancheng/tinyos/moteiv/tinyos-1.x/tos/interfaces/SplitControl.nc"
inline static result_t CC2420RadioM$CC2420SplitControl$stop(void ){
#line 93
  unsigned char __nesc_result;
#line 93

#line 93
  __nesc_result = CC2420ControlM$SplitControl$stop();
#line 93

#line 93
  return __nesc_result;
#line 93
}
#line 93
# 49 "/Users/jingyuancheng/tinyos/moteiv/tos/platform/msp430/timer/MSP430AlarmC.nc"
static inline result_t /*CC2420RadioC.BackoffAlarm32khzC.MSP430Alarm*/MSP430AlarmC$1$Init$stop(void )
#line 49
{
#line 49
  return SUCCESS;
}

# 78 "/Users/jingyuancheng/tinyos/moteiv/tinyos-1.x/tos/interfaces/StdControl.nc"
inline static result_t CC2420RadioM$TimerControl$stop(void ){
#line 78
  unsigned char __nesc_result;
#line 78

#line 78
  __nesc_result = /*CC2420RadioC.BackoffAlarm32khzC.MSP430Alarm*/MSP430AlarmC$1$Init$stop();
#line 78

#line 78
  return __nesc_result;
#line 78
}
#line 78
# 59 "/Users/jingyuancheng/tinyos/moteiv/tinyos-1.x/tos/lib/CC2420Radio/HPLCC2420Interrupt.nc"
inline static result_t CC2420RadioM$FIFOP$disable(void ){
#line 59
  unsigned char __nesc_result;
#line 59

#line 59
  __nesc_result = HPLCC2420InterruptM$FIFOP$disable();
#line 59

#line 59
  return __nesc_result;
#line 59
}
#line 59
# 49 "/Users/jingyuancheng/tinyos/moteiv/tos/lib/sched/TaskBasic.nc"
inline static result_t CC2420RadioM$taskShutdownRequest$postTask(void ){
#line 49
  unsigned char __nesc_result;
#line 49

#line 49
  __nesc_result = SchedulerBasicP$TaskBasic$postTask(CC2420RadioM$taskShutdownRequest);
#line 49

#line 49
  return __nesc_result;
#line 49
}
#line 49
# 250 "/Users/jingyuancheng/tinyos/moteiv/tos/lib/CC2420Radio/CC2420RadioM.nc"
static inline void CC2420RadioM$taskShutdownRequest$runTask(void )
#line 250
{
  bool bShutdown = FALSE;

#line 252
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 252
    {

      if ((
#line 253
      CC2420RadioM$stateRadio != CC2420RadioM$IDLE_STATE || 
      CC2420RadioM$bPacketReceiving) || 
      CC2420RadioM$m_rxFifoCount > 0) {

          CC2420RadioM$taskShutdownRequest$postTask();
        }
      else {
          bShutdown = TRUE;
        }
    }
#line 262
    __nesc_atomic_end(__nesc_atomic); }

  if (bShutdown) {
      { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 265
        CC2420RadioM$stateRadio = CC2420RadioM$DISABLED_STATE;
#line 265
        __nesc_atomic_end(__nesc_atomic); }

      CC2420RadioM$SFD$disable();
      CC2420RadioM$FIFOP$disable();
      CC2420RadioM$TimerControl$stop();
      CC2420RadioM$CC2420SplitControl$stop();
    }
}

#line 347
static inline void CC2420RadioM$startRadio$runTask(void )
#line 347
{
  result_t success = FAIL;

#line 349
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 349
    {
      if (CC2420RadioM$stateRadio == CC2420RadioM$DISABLED_STATE_STARTTASK) {
          CC2420RadioM$stateRadio = CC2420RadioM$DISABLED_STATE;
          success = SUCCESS;
        }
    }
#line 354
    __nesc_atomic_end(__nesc_atomic); }

  if (success == SUCCESS) {
    CC2420RadioM$SplitControl$start();
    }
}

# 485 "/Users/jingyuancheng/tinyos/moteiv/tos/lib/CC2420Radio/CC2420ControlM.nc"
static inline result_t CC2420ControlM$HPLChipconRAM$readDone(uint16_t addr, uint8_t length, uint8_t *buffer)
#line 485
{
  return SUCCESS;
}

# 114 "/Users/jingyuancheng/tinyos/moteiv/tos/lib/sp/cc2420/CC2420TimeStampingM.nc"
static inline result_t CC2420TimeStampingM$HPLCC2420RAM$readDone(uint16_t addr, uint8_t length, uint8_t *buffer)
#line 114
{
  return SUCCESS;
}

# 66 "/Users/jingyuancheng/tinyos/moteiv/tos/lib/CC2420Radio/HPLCC2420RAM.nc"
inline static result_t HPLCC2420M$HPLCC2420RAM$readDone(uint16_t addr, uint8_t length, uint8_t *buffer){
#line 66
  unsigned char __nesc_result;
#line 66

#line 66
  __nesc_result = CC2420TimeStampingM$HPLCC2420RAM$readDone(addr, length, buffer);
#line 66
  __nesc_result = rcombine(__nesc_result, CC2420ControlM$HPLChipconRAM$readDone(addr, length, buffer));
#line 66

#line 66
  return __nesc_result;
#line 66
}
#line 66
# 208 "/Users/jingyuancheng/tinyos/moteiv/tos/platform/tmote/HPLCC2420M.nc"
static inline void HPLCC2420M$signalRAMRd$runTask(void )
#line 208
{
  HPLCC2420M$HPLCC2420RAM$readDone(HPLCC2420M$rxramaddr, HPLCC2420M$rxramlen, HPLCC2420M$rxrambuf);
}

# 489 "/Users/jingyuancheng/tinyos/moteiv/tos/lib/CC2420Radio/CC2420ControlM.nc"
static inline result_t CC2420ControlM$HPLChipconRAM$writeDone(uint16_t addr, uint8_t length, uint8_t *buffer)
#line 489
{
  return SUCCESS;
}

# 118 "/Users/jingyuancheng/tinyos/moteiv/tos/lib/sp/cc2420/CC2420TimeStampingM.nc"
static inline result_t CC2420TimeStampingM$HPLCC2420RAM$writeDone(uint16_t addr, uint8_t length, uint8_t *buffer)
#line 118
{
  return SUCCESS;
}

# 51 "/Users/jingyuancheng/tinyos/moteiv/tos/lib/CC2420Radio/HPLCC2420RAM.nc"
inline static result_t HPLCC2420M$HPLCC2420RAM$writeDone(uint16_t addr, uint8_t length, uint8_t *buffer){
#line 51
  unsigned char __nesc_result;
#line 51

#line 51
  __nesc_result = CC2420TimeStampingM$HPLCC2420RAM$writeDone(addr, length, buffer);
#line 51
  __nesc_result = rcombine(__nesc_result, CC2420ControlM$HPLChipconRAM$writeDone(addr, length, buffer));
#line 51

#line 51
  return __nesc_result;
#line 51
}
#line 51
# 247 "/Users/jingyuancheng/tinyos/moteiv/tos/platform/tmote/HPLCC2420M.nc"
static inline void HPLCC2420M$signalRAMWr$runTask(void )
#line 247
{
  HPLCC2420M$HPLCC2420RAM$writeDone(HPLCC2420M$ramaddr, HPLCC2420M$ramlen, HPLCC2420M$rambuf);
}

# 159 "/Users/jingyuancheng/tinyos/moteiv/tos/lib/resource/FcfsArbiterP.nc"
static inline void /*MSP430ArbiterUSART0C.ArbiterC.ArbiterP*/FcfsArbiterP$1$ResourceCmd$release(uint8_t id)
#line 159
{
  /*MSP430ArbiterUSART0C.ArbiterC.ArbiterP*/FcfsArbiterP$1$Resource$release(id);
}

# 75 "/Users/jingyuancheng/tinyos/moteiv/tos/lib/resource/ResourceCmd.nc"
inline static void CC2420RadioM$CmdReceive$release(void ){
#line 75
  /*MSP430ArbiterUSART0C.ArbiterC.ArbiterP*/FcfsArbiterP$1$ResourceCmd$release(/*CC2420RadioC.CmdReceiveC.ResourceC.ResourceUSARTP*/MSP430ResourceUSART0P$6$ID);
#line 75
}
#line 75
# 49 "/Users/jingyuancheng/tinyos/moteiv/tos/lib/sched/TaskBasic.nc"
inline static result_t CC2420RadioM$PacketRcvd$postTask(void ){
#line 49
  unsigned char __nesc_result;
#line 49

#line 49
  __nesc_result = SchedulerBasicP$TaskBasic$postTask(CC2420RadioM$PacketRcvd);
#line 49

#line 49
  return __nesc_result;
#line 49
}
#line 49
# 48 "/Users/jingyuancheng/tinyos/moteiv/tos/lib/CC2420Radio/HPLCC2420.nc"
inline static uint8_t CC2420RadioM$HPLChipcon$cmd(uint8_t rh, uint8_t addr){
#line 48
  unsigned char __nesc_result;
#line 48

#line 48
  __nesc_result = HPLCC2420M$HPLCC2420$cmd(rh, addr);
#line 48

#line 48
  return __nesc_result;
#line 48
}
#line 48
# 137 "/Users/jingyuancheng/tinyos/moteiv/tos/lib/util/circularQueue.h"
static inline result_t cqueue_popFront(CircularQueue_t *cq)
{
  if (cqueue_isEmpty(cq)) {
    return FAIL;
    }
  if (cq->front == cq->back) 
    {
      cq->front = cq->size;
      cq->back = cq->size;
      return FAIL;
    }

  cq->front = cqueue_privDec(cq, cq->front);
  return SUCCESS;
}

# 12 "/Users/jingyuancheng/tinyos/moteiv/tos/lib/CC2420Radio/byteorder.h"
static __inline int is_host_lsb()
{
  const uint8_t n[2] = { 1, 0 };

#line 15
  return * (uint16_t *)n == 1;
}






static __inline uint16_t fromLSB16(uint16_t a)
{
  return is_host_lsb() ? a : (a << 8) | (a >> 8);
}

# 801 "/Users/jingyuancheng/tinyos/moteiv/tos/lib/CC2420Radio/CC2420RadioM.nc"
static inline result_t CC2420RadioM$doRXFIFODoneBody(uint8_t rh, uint8_t length, uint8_t *data)
#line 801
{





  uint8_t currentstate;

#line 808
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 808
    {
      currentstate = CC2420RadioM$stateRadio;
    }
#line 810
    __nesc_atomic_end(__nesc_atomic); }




  if (((
#line 814
  !TOSH_READ_CC_FIFO_PIN() && !TOSH_READ_CC_FIFOP_PIN())
   || length == 0) || length > MSG_DATA_SIZE) {
      CC2420RadioM$flushRXFIFO(rh);
      return SUCCESS;
    }

  CC2420RadioM$rxbufptr = (TOS_MsgPtr )data;




  if (
#line 823
  currentstate == CC2420RadioM$POST_TX_STATE && (
  CC2420RadioM$rxbufptr->fcfhi & 0x07) == 0x02 && 
  CC2420RadioM$rxbufptr->dsn == CC2420RadioM$currentDSN && (
  data[length - 1] >> 7) == 1) {
      { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 827
        {
          CC2420RadioM$txbufptr->ack = 1;
          CC2420RadioM$txbufptr->strength = data[length - 2];
          CC2420RadioM$txbufptr->lqi = data[length - 1] & 0x7F;

          CC2420RadioM$stateRadio = CC2420RadioM$POST_TX_ACK_STATE;
          CC2420RadioM$bPacketReceiving = FALSE;
        }
#line 834
        __nesc_atomic_end(__nesc_atomic); }
      if (!CC2420RadioM$PacketSent$postTask()) {
        CC2420RadioM$sendFailedAsync();
        }






      if (TOSH_READ_CC_FIFO_PIN()) {
          CC2420RadioM$CmdReceive$deferRequest();
          return SUCCESS;
        }

      CC2420RadioM$flushRXFIFO(rh);
      return SUCCESS;
    }




  if ((CC2420RadioM$rxbufptr->fcfhi & 0x07) != 0x01 || 
  CC2420RadioM$rxbufptr->fcflo != 0x08) {
      CC2420RadioM$flushRXFIFO(rh);
      return SUCCESS;
    }
  CC2420RadioM$rxbufptr->length = CC2420RadioM$rxbufptr->length - MSG_HEADER_SIZE - MSG_FOOTER_SIZE;

  if (CC2420RadioM$rxbufptr->length > 66) {
      CC2420RadioM$flushRXFIFO(rh);
      return SUCCESS;
    }


  CC2420RadioM$rxbufptr->addr = fromLSB16(CC2420RadioM$rxbufptr->addr);



  CC2420RadioM$rxbufptr->crc = data[length - 1] >> 7;

  CC2420RadioM$rxbufptr->strength = data[length - 2];

  CC2420RadioM$rxbufptr->lqi = data[length - 1] & 0x7F;


  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 880
    {
      if (!cqueue_isEmpty(&CC2420RadioM$m_timestampQueue)) {
        CC2420RadioM$rxbufptr->time = CC2420RadioM$m_timestamps[CC2420RadioM$m_timestampQueue.front];
        }
#line 883
      cqueue_popFront(&CC2420RadioM$m_timestampQueue);
    }
#line 884
    __nesc_atomic_end(__nesc_atomic); }




  if (
#line 887
  CC2420RadioM$rxbufptr->fcfhi & (1 << 5) && 
  CC2420RadioM$rxbufptr->crc && 
  CC2420RadioM$rxbufptr->group == TOS_AM_GROUP && 
  CC2420RadioM$rxbufptr->addr == TOS_LOCAL_ADDRESS) {
      CC2420RadioM$HPLChipcon$cmd(rh, 0x0A);
    }

  CC2420RadioM$PacketRcvd$postTask();






  return SUCCESS;
}

static inline result_t CC2420RadioM$HPLChipconFIFO$RXFIFODone(uint8_t length, uint8_t *data)
#line 904
{
  CC2420RadioM$doRXFIFODoneBody(CC2420RadioM$rh_receive, length, data);
  CC2420RadioM$CmdReceive$release();
  return SUCCESS;
}

# 52 "/Users/jingyuancheng/tinyos/moteiv/tos/lib/CC2420Radio/HPLCC2420FIFO.nc"
inline static result_t HPLCC2420M$HPLCC2420FIFO$RXFIFODone(uint8_t length, uint8_t *data){
#line 52
  unsigned char __nesc_result;
#line 52

#line 52
  __nesc_result = CC2420RadioM$HPLChipconFIFO$RXFIFODone(length, data);
#line 52

#line 52
  return __nesc_result;
#line 52
}
#line 52
# 128 "/Users/jingyuancheng/tinyos/moteiv/tos/platform/tmote/HPLCC2420M.nc"
static inline void HPLCC2420M$release(void )
#line 128
{
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 129
    {
      HPLCC2420M$f_busy = HPLCC2420M$IDLE;
    }
#line 131
    __nesc_atomic_end(__nesc_atomic); }
}

#line 279
static inline void HPLCC2420M$signalRXFIFO$runTask(void )
#line 279
{
  if (HPLCC2420M$f_busy == HPLCC2420M$BUSY_RX) {
      uint8_t _rxlen;
      uint8_t *_rxbuf;

      { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 284
        {
          _rxlen = HPLCC2420M$rxlen;
          _rxbuf = HPLCC2420M$rxbuf;
        }
#line 287
        __nesc_atomic_end(__nesc_atomic); }

      HPLCC2420M$release();
      HPLCC2420M$HPLCC2420FIFO$RXFIFODone(_rxlen, _rxbuf);
    }
}

# 75 "/Users/jingyuancheng/tinyos/moteiv/tos/lib/resource/ResourceCmd.nc"
inline static void CC2420RadioM$CmdTransmit$release(void ){
#line 75
  /*MSP430ArbiterUSART0C.ArbiterC.ArbiterP*/FcfsArbiterP$1$ResourceCmd$release(/*CC2420RadioC.CmdTransmitC.ResourceC.ResourceUSARTP*/MSP430ResourceUSART0P$7$ID);
#line 75
}
#line 75
# 925 "/Users/jingyuancheng/tinyos/moteiv/tos/lib/CC2420Radio/CC2420RadioM.nc"
static inline result_t CC2420RadioM$HPLChipconFIFO$TXFIFODone(uint8_t length, uint8_t *data)
#line 925
{
  if (CC2420RadioM$bShutdownRequest) {
      CC2420RadioM$sendFailedAsync();
    }
  else {
      CC2420RadioM$tryToSend(CC2420RadioM$rh_transmit);
    }
  CC2420RadioM$CmdTransmit$release();
  return SUCCESS;
}

# 63 "/Users/jingyuancheng/tinyos/moteiv/tos/lib/CC2420Radio/HPLCC2420FIFO.nc"
inline static result_t HPLCC2420M$HPLCC2420FIFO$TXFIFODone(uint8_t length, uint8_t *data){
#line 63
  unsigned char __nesc_result;
#line 63

#line 63
  __nesc_result = CC2420RadioM$HPLChipconFIFO$TXFIFODone(length, data);
#line 63

#line 63
  return __nesc_result;
#line 63
}
#line 63
# 332 "/Users/jingyuancheng/tinyos/moteiv/tos/platform/tmote/HPLCC2420M.nc"
static inline void HPLCC2420M$signalTXFIFO$runTask(void )
#line 332
{
  if (HPLCC2420M$f_busy == HPLCC2420M$BUSY_TX) {
      uint8_t _txlen;
      uint8_t *_txbuf;

      { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 337
        {
          _txlen = HPLCC2420M$txlen;
          _txbuf = HPLCC2420M$txbuf;
        }
#line 340
        __nesc_atomic_end(__nesc_atomic); }

      HPLCC2420M$release();
      HPLCC2420M$HPLCC2420FIFO$TXFIFODone(_txlen, _txbuf);
    }
}

# 60 "/Users/jingyuancheng/tinyos/moteiv/tos/lib/timer/Timer2.nc"
inline static void RefVoltM$SwitchOnTimer$startOneShot(uint32_t dt){
#line 60
  /*HalTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC$0$Timer$startOneShot(/*RefVoltC.Timer1*/TimerMilliC$2$TIMER_ID, dt);
#line 60
}
#line 60
# 130 "/Users/jingyuancheng/tinyos/moteiv/tos/platform/msp430/adc/RefVoltM.nc"
static inline void RefVoltM$switchOnDelay$runTask(void )
#line 130
{
  RefVoltM$SwitchOnTimer$startOneShot(17);
}

# 60 "/Users/jingyuancheng/tinyos/moteiv/tos/lib/timer/Timer2.nc"
inline static void RefVoltM$SwitchOffTimer$startOneShot(uint32_t dt){
#line 60
  /*HalTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC$0$Timer$startOneShot(/*RefVoltC.Timer2*/TimerMilliC$3$TIMER_ID, dt);
#line 60
}
#line 60
# 192 "/Users/jingyuancheng/tinyos/moteiv/tos/platform/msp430/adc/RefVoltM.nc"
static inline void RefVoltM$switchOffDelay$runTask(void )
#line 192
{
  if (RefVoltM$switchOff == TRUE) {
    RefVoltM$SwitchOffTimer$startOneShot(100);
    }
}

#line 197
static inline void RefVoltM$switchOffRetry$runTask(void )
#line 197
{
  if (RefVoltM$switchOff == TRUE) {
    RefVoltM$SwitchOffTimer$startOneShot(5);
    }
}

# 230 "/Users/jingyuancheng/tinyos/moteiv/tos/lib/resource/FcfsArbiterP.nc"
static inline void /*MSP430ArbiterUSART0C.ArbiterC.ArbiterP*/FcfsArbiterP$1$Resource$default$granted(uint8_t id)
#line 230
{
}

# 73 "/Users/jingyuancheng/tinyos/moteiv/tos/lib/resource/Resource.nc"
inline static void /*MSP430ArbiterUSART0C.ArbiterC.ArbiterP*/FcfsArbiterP$1$Resource$granted(uint8_t arg_0x101b3c648){
#line 73
    /*MSP430ArbiterUSART0C.ArbiterC.ArbiterP*/FcfsArbiterP$1$Resource$default$granted(arg_0x101b3c648);
#line 73
}
#line 73
# 940 "/Users/jingyuancheng/tinyos/moteiv/tos/lib/CC2420Radio/CC2420RadioM.nc"
static inline void CC2420RadioM$MacControl$enableAck(void )
#line 940
{
}

# 22 "/Users/jingyuancheng/tinyos/moteiv/tos/lib/CC2420Radio/MacControl.nc"
inline static void CC2420AlwaysOnM$MacControl$enableAck(void ){
#line 22
  CC2420RadioM$MacControl$enableAck();
#line 22
}
#line 22
# 181 "/Users/jingyuancheng/tinyos/moteiv/tos/lib/sp/cc2420/CC2420AlwaysOnM.nc"
static inline result_t CC2420AlwaysOnM$RadioControl$startDone(void )
#line 181
{
  CC2420AlwaysOnM$MacControl$enableAck();
  return SUCCESS;
}

# 85 "/Users/jingyuancheng/tinyos/moteiv/tinyos-1.x/tos/interfaces/SplitControl.nc"
inline static result_t CC2420RadioM$SplitControl$startDone(void ){
#line 85
  unsigned char __nesc_result;
#line 85

#line 85
  __nesc_result = CC2420AlwaysOnM$RadioControl$startDone();
#line 85

#line 85
  return __nesc_result;
#line 85
}
#line 85
# 43 "/Users/jingyuancheng/tinyos/moteiv/tinyos-1.x/tos/lib/CC2420Radio/HPLCC2420Interrupt.nc"
inline static result_t CC2420RadioM$FIFOP$startWait(bool low_to_high){
#line 43
  unsigned char __nesc_result;
#line 43

#line 43
  __nesc_result = HPLCC2420InterruptM$FIFOP$startWait(low_to_high);
#line 43

#line 43
  return __nesc_result;
#line 43
}
#line 43
# 58 "/Users/jingyuancheng/tinyos/moteiv/tos/lib/resource/ResourceCmdAsync.nc"
inline static void CC2420ControlM$CmdCmds$request(uint8_t rh){
#line 58
  /*MSP430ArbiterUSART0C.ArbiterC.ArbiterP*/FcfsArbiterP$1$ResourceCmdAsync$request(/*CC2420RadioC.CmdCmds.ResourceC.ResourceUSARTP*/MSP430ResourceUSART0P$4$ID, rh);
#line 58
}
#line 58
# 533 "/Users/jingyuancheng/tinyos/moteiv/tos/lib/CC2420Radio/CC2420ControlM.nc"
static inline void CC2420ControlM$doCmds(uint8_t rh)
#line 533
{
  CC2420ControlM$CmdCmds$request(rh);
}

#line 362
static inline result_t CC2420ControlM$CC2420Control$RxMode(uint8_t rh)
#line 362
{
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 363
    CC2420ControlM$cmds.rxtxmode = CC2420ControlM$CMD_SRXON;
#line 363
    __nesc_atomic_end(__nesc_atomic); }
  CC2420ControlM$doCmds(rh);
  return SUCCESS;
}

# 172 "/Users/jingyuancheng/tinyos/moteiv/tos/lib/CC2420Radio/CC2420Control.nc"
inline static result_t CC2420RadioM$CC2420Control$RxMode(uint8_t rh){
#line 172
  unsigned char __nesc_result;
#line 172

#line 172
  __nesc_result = CC2420ControlM$CC2420Control$RxMode(rh);
#line 172

#line 172
  return __nesc_result;
#line 172
}
#line 172
# 402 "/Users/jingyuancheng/tinyos/moteiv/tos/lib/CC2420Radio/CC2420RadioM.nc"
static inline result_t CC2420RadioM$CC2420SplitControl$startDone(void )
#line 402
{
  uint8_t chkstateRadio;

  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 405
    chkstateRadio = CC2420RadioM$stateRadio;
#line 405
    __nesc_atomic_end(__nesc_atomic); }

  if (chkstateRadio == CC2420RadioM$WARMUP_STATE) {

      CC2420RadioM$cc2420_laston = CC2420RadioM$Counter32khz$get();

      CC2420RadioM$CC2420Control$RxMode(RESOURCE_NONE);

      CC2420RadioM$FIFOP$startWait(FALSE);

      CC2420RadioM$SFD$enableCapture(TRUE);

      { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 417
        CC2420RadioM$stateRadio = CC2420RadioM$IDLE_STATE;
#line 417
        __nesc_atomic_end(__nesc_atomic); }
    }
  CC2420RadioM$SplitControl$startDone();
  return SUCCESS;
}

# 85 "/Users/jingyuancheng/tinyos/moteiv/tinyos-1.x/tos/interfaces/SplitControl.nc"
inline static result_t CC2420ControlM$SplitControl$startDone(void ){
#line 85
  unsigned char __nesc_result;
#line 85

#line 85
  __nesc_result = CC2420RadioM$CC2420SplitControl$startDone();
#line 85

#line 85
  return __nesc_result;
#line 85
}
#line 85
# 75 "/Users/jingyuancheng/tinyos/moteiv/tos/lib/resource/ResourceCmd.nc"
inline static void CC2420ControlM$CmdCCAFired$release(void ){
#line 75
  /*MSP430ArbiterUSART0C.ArbiterC.ArbiterP*/FcfsArbiterP$1$ResourceCmd$release(/*CC2420RadioC.CmdCCAFiredC.ResourceC.ResourceUSARTP*/MSP430ResourceUSART0P$0$ID);
#line 75
}
#line 75
# 294 "/Users/jingyuancheng/tinyos/moteiv/tos/lib/CC2420Radio/CC2420ControlM.nc"
static inline result_t CC2420ControlM$CC2420Control$TuneManual(uint8_t rh, uint16_t DesiredFreq)
#line 294
{
  int fsctrl;

  fsctrl = DesiredFreq - 2048;
  CC2420ControlM$gCurrentParameters[CP_FSCTRL] = (CC2420ControlM$gCurrentParameters[CP_FSCTRL] & 0xfc00) | (fsctrl << 0);
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 299
    CC2420ControlM$cmds.freqselect = 1;
#line 299
    __nesc_atomic_end(__nesc_atomic); }
  CC2420ControlM$doCmds(rh);
  return SUCCESS;
}

# 18 "/Users/jingyuancheng/tinyos/moteiv/tos/lib/CC2420Radio/byteorder.h"
static __inline uint16_t toLSB16(uint16_t a)
{
  return is_host_lsb() ? a : (a << 8) | (a >> 8);
}

# 478 "/Users/jingyuancheng/tinyos/moteiv/tos/lib/CC2420Radio/CC2420ControlM.nc"
static inline result_t CC2420ControlM$CC2420Control$setShortAddress(uint8_t rh, uint16_t addr)
#line 478
{
  CC2420ControlM$shortAddress = toLSB16(addr);
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 480
    CC2420ControlM$cmds.setshortaddress = 1;
#line 480
    __nesc_atomic_end(__nesc_atomic); }
  CC2420ControlM$doCmds(rh);
  return SUCCESS;
}

# 71 "/Users/jingyuancheng/tinyos/moteiv/tos/lib/CC2420Radio/HPLCC2420.nc"
inline static uint16_t CC2420ControlM$HPLChipcon$read(uint8_t rh, uint8_t addr){
#line 71
  unsigned int __nesc_result;
#line 71

#line 71
  __nesc_result = HPLCC2420M$HPLCC2420$read(rh, addr);
#line 71

#line 71
  return __nesc_result;
#line 71
}
#line 71
# 95 "/Users/jingyuancheng/tinyos/moteiv/tos/lib/CC2420Radio/CC2420ControlM.nc"
static inline bool CC2420ControlM$SetRegs(uint8_t rh)
#line 95
{
  uint16_t data;

  CC2420ControlM$HPLChipcon$write(rh, 0x10, CC2420ControlM$gCurrentParameters[CP_MAIN]);
  CC2420ControlM$HPLChipcon$write(rh, 0x11, CC2420ControlM$gCurrentParameters[CP_MDMCTRL0]);
  data = CC2420ControlM$HPLChipcon$read(rh, 0x11);
  if (data != CC2420ControlM$gCurrentParameters[CP_MDMCTRL0]) {
#line 101
    return FALSE;
    }
  CC2420ControlM$HPLChipcon$write(rh, 0x12, CC2420ControlM$gCurrentParameters[CP_MDMCTRL1]);
  CC2420ControlM$HPLChipcon$write(rh, 0x13, CC2420ControlM$gCurrentParameters[CP_RSSI]);
  CC2420ControlM$HPLChipcon$write(rh, 0x14, CC2420ControlM$gCurrentParameters[CP_SYNCWORD]);
  CC2420ControlM$HPLChipcon$write(rh, 0x15, CC2420ControlM$gCurrentParameters[CP_TXCTRL]);
  CC2420ControlM$HPLChipcon$write(rh, 0x16, CC2420ControlM$gCurrentParameters[CP_RXCTRL0]);
  CC2420ControlM$HPLChipcon$write(rh, 0x17, CC2420ControlM$gCurrentParameters[CP_RXCTRL1]);
  CC2420ControlM$HPLChipcon$write(rh, 0x18, CC2420ControlM$gCurrentParameters[CP_FSCTRL]);

  CC2420ControlM$HPLChipcon$write(rh, 0x19, CC2420ControlM$gCurrentParameters[CP_SECCTRL0]);
  CC2420ControlM$HPLChipcon$write(rh, 0x1A, CC2420ControlM$gCurrentParameters[CP_SECCTRL1]);
  CC2420ControlM$HPLChipcon$write(rh, 0x1C, CC2420ControlM$gCurrentParameters[CP_IOCFG0]);
  CC2420ControlM$HPLChipcon$write(rh, 0x1D, CC2420ControlM$gCurrentParameters[CP_IOCFG1]);

  CC2420ControlM$HPLChipcon$cmd(rh, 0x09);
  CC2420ControlM$HPLChipcon$cmd(rh, 0x08);

  return TRUE;
}

#line 493
static inline void CC2420ControlM$CmdCCAFired$granted(uint8_t rh)
#line 493
{

  CC2420ControlM$HPLChipcon$write(rh, 0x1D, 0);

  CC2420ControlM$SetRegs(rh);
  CC2420ControlM$CC2420Control$setShortAddress(rh, TOS_LOCAL_ADDRESS);
  CC2420ControlM$CC2420Control$TuneManual(rh, ((CC2420ControlM$gCurrentParameters[CP_FSCTRL] << 0) & 0x1FF) + 2048);

  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 501
    CC2420ControlM$state = CC2420ControlM$START_STATE_DONE;
#line 501
    __nesc_atomic_end(__nesc_atomic); }
  CC2420ControlM$CmdCCAFired$release();
  CC2420ControlM$SplitControl$startDone();
}

# 180 "/Users/jingyuancheng/tinyos/moteiv/tos/lib/sp/cc2420/CC2420AlwaysOnM.nc"
static inline result_t CC2420AlwaysOnM$RadioControl$initDone(void )
#line 180
{
#line 180
  return SUCCESS;
}

# 70 "/Users/jingyuancheng/tinyos/moteiv/tinyos-1.x/tos/interfaces/SplitControl.nc"
inline static result_t CC2420RadioM$SplitControl$initDone(void ){
#line 70
  unsigned char __nesc_result;
#line 70

#line 70
  __nesc_result = CC2420AlwaysOnM$RadioControl$initDone();
#line 70

#line 70
  return __nesc_result;
#line 70
}
#line 70
# 304 "/Users/jingyuancheng/tinyos/moteiv/tos/lib/CC2420Radio/CC2420RadioM.nc"
static inline result_t CC2420RadioM$CC2420SplitControl$initDone(void )
#line 304
{
  return CC2420RadioM$SplitControl$initDone();
}

# 70 "/Users/jingyuancheng/tinyos/moteiv/tinyos-1.x/tos/interfaces/SplitControl.nc"
inline static result_t CC2420ControlM$SplitControl$initDone(void ){
#line 70
  unsigned char __nesc_result;
#line 70

#line 70
  __nesc_result = CC2420RadioM$CC2420SplitControl$initDone();
#line 70

#line 70
  return __nesc_result;
#line 70
}
#line 70
# 75 "/Users/jingyuancheng/tinyos/moteiv/tos/lib/resource/ResourceCmd.nc"
inline static void CC2420ControlM$CmdSplitControlInit$release(void ){
#line 75
  /*MSP430ArbiterUSART0C.ArbiterC.ArbiterP*/FcfsArbiterP$1$ResourceCmd$release(/*CC2420RadioC.CmdSplitControlInitC.ResourceC.ResourceUSARTP*/MSP430ResourceUSART0P$1$ID);
#line 75
}
#line 75
# 20 "/Users/jingyuancheng/tinyos/moteiv/tos/platform/tmote/hardware.h"
static inline void TOSH_MAKE_RADIO_CSN_OUTPUT()
#line 20
{
#line 20
  static volatile uint8_t r __asm ("0x001E");

#line 20
  r |= 1 << 2;
}

#line 20
static inline void TOSH_SET_RADIO_CSN_PIN()
#line 20
{
#line 20
  static volatile uint8_t r __asm ("0x001D");

#line 20
  r |= 1 << 2;
}


static inline void TOSH_MAKE_RADIO_SFD_INPUT()
#line 24
{
#line 24
  static volatile uint8_t r __asm ("0x001E");

#line 24
  r &= ~(1 << 1);
}

# 32 "/Users/jingyuancheng/tinyos/moteiv/tinyos-1.x/tos/platform/msp430/MSP430GeneralIO.h"
static inline void TOSH_MAKE_PORT14_INPUT()
#line 32
{
#line 32
  static volatile uint8_t r __asm ("0x0022");

#line 32
  r &= ~(1 << 4);
}

# 343 "/Users/jingyuancheng/tinyos/moteiv/tos/platform/msp430/MSP430InterruptM.nc"
static inline void MSP430InterruptM$Port14$makeInput(void )
#line 343
{
#line 343
  TOSH_MAKE_PORT14_INPUT();
}

# 69 "/Users/jingyuancheng/tinyos/moteiv/tos/platform/msp430/MSP430Interrupt.nc"
inline static void HPLCC2420InterruptM$CCAInterrupt$makeInput(void ){
#line 69
  MSP430InterruptM$Port14$makeInput();
#line 69
}
#line 69
# 31 "/Users/jingyuancheng/tinyos/moteiv/tinyos-1.x/tos/platform/msp430/MSP430GeneralIO.h"
static inline void TOSH_MAKE_PORT13_INPUT()
#line 31
{
#line 31
  static volatile uint8_t r __asm ("0x0022");

#line 31
  r &= ~(1 << 3);
}

# 342 "/Users/jingyuancheng/tinyos/moteiv/tos/platform/msp430/MSP430InterruptM.nc"
static inline void MSP430InterruptM$Port13$makeInput(void )
#line 342
{
#line 342
  TOSH_MAKE_PORT13_INPUT();
}

# 69 "/Users/jingyuancheng/tinyos/moteiv/tos/platform/msp430/MSP430Interrupt.nc"
inline static void HPLCC2420InterruptM$FIFOInterrupt$makeInput(void ){
#line 69
  MSP430InterruptM$Port13$makeInput();
#line 69
}
#line 69
# 28 "/Users/jingyuancheng/tinyos/moteiv/tinyos-1.x/tos/platform/msp430/MSP430GeneralIO.h"
static inline void TOSH_MAKE_PORT10_INPUT()
#line 28
{
#line 28
  static volatile uint8_t r __asm ("0x0022");

#line 28
  r &= ~(1 << 0);
}

# 339 "/Users/jingyuancheng/tinyos/moteiv/tos/platform/msp430/MSP430InterruptM.nc"
static inline void MSP430InterruptM$Port10$makeInput(void )
#line 339
{
#line 339
  TOSH_MAKE_PORT10_INPUT();
}

# 69 "/Users/jingyuancheng/tinyos/moteiv/tos/platform/msp430/MSP430Interrupt.nc"
inline static void HPLCC2420InterruptM$FIFOPInterrupt$makeInput(void ){
#line 69
  MSP430InterruptM$Port10$makeInput();
#line 69
}
#line 69
# 60 "/Users/jingyuancheng/tinyos/moteiv/tos/platform/tmote/HPLCC2420InterruptM.nc"
static inline result_t HPLCC2420InterruptM$Init$init(void )
#line 60
{
  HPLCC2420InterruptM$FIFOPInterrupt$makeInput();
  HPLCC2420InterruptM$FIFOInterrupt$makeInput();
  HPLCC2420InterruptM$CCAInterrupt$makeInput();
  TOSH_MAKE_RADIO_SFD_INPUT();
  return SUCCESS;
}

# 46 "/Users/jingyuancheng/tinyos/moteiv/tos/lib/sched/Init.nc"
inline static result_t HPLCC2420M$InterruptInit$init(void ){
#line 46
  unsigned char __nesc_result;
#line 46

#line 46
  __nesc_result = HPLCC2420InterruptM$Init$init();
#line 46

#line 46
  return __nesc_result;
#line 46
}
#line 46
# 87 "/Users/jingyuancheng/tinyos/moteiv/tos/platform/tmote/HPLCC2420M.nc"
static inline result_t HPLCC2420M$StdControl$init(void )
#line 87
{
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 88
    {
      HPLCC2420M$f_enabled = FALSE;
      HPLCC2420M$f_busy = HPLCC2420M$IDLE;
      HPLCC2420M$InterruptInit$init();
    }
#line 92
    __nesc_atomic_end(__nesc_atomic); }

  TOSH_SET_RADIO_CSN_PIN();
  TOSH_MAKE_RADIO_CSN_OUTPUT();
  return SUCCESS;
}

# 63 "/Users/jingyuancheng/tinyos/moteiv/tinyos-1.x/tos/interfaces/StdControl.nc"
inline static result_t CC2420ControlM$HPLChipconControl$init(void ){
#line 63
  unsigned char __nesc_result;
#line 63

#line 63
  __nesc_result = HPLCC2420M$StdControl$init();
#line 63

#line 63
  return __nesc_result;
#line 63
}
#line 63
# 129 "/Users/jingyuancheng/tinyos/moteiv/tos/lib/CC2420Radio/CC2420ControlM.nc"
static inline void CC2420ControlM$CmdSplitControlInit$granted(uint8_t rh)
#line 129
{

  CC2420ControlM$HPLChipconControl$init();


  CC2420ControlM$gCurrentParameters[CP_MAIN] = 0xf800;
  CC2420ControlM$gCurrentParameters[CP_MDMCTRL0] = ((((0 << 11) | (
  2 << 8)) | (3 << 6)) | (
  1 << 5)) | (2 << 0);

  CC2420ControlM$gCurrentParameters[CP_MDMCTRL1] = 20 << 6;

  CC2420ControlM$gCurrentParameters[CP_RSSI] = 0xE080;
  CC2420ControlM$gCurrentParameters[CP_SYNCWORD] = 0xA70F;
  CC2420ControlM$gCurrentParameters[CP_TXCTRL] = ((((2 << 14) | (
  1 << 13)) | (3 << 6)) | (
  1 << 5)) | (CC2420_RFPOWER << 0);

  CC2420ControlM$gCurrentParameters[CP_RXCTRL0] = (((((1 << 12) | (
  2 << 8)) | (3 << 6)) | (
  2 << 4)) | (1 << 2)) | (
  1 << 0);

  CC2420ControlM$gCurrentParameters[CP_RXCTRL1] = ((((((1 << 13) | (
  1 << 11)) | (1 << 9)) | (
  1 << 6)) | (1 << 4)) | (
  1 << 2)) | (2 << 0);

  CC2420ControlM$gCurrentParameters[CP_FSCTRL] = (1 << 14) | ((
  357 + 5 * (CC2420_CHANNEL - 11)) << 0);

  CC2420ControlM$gCurrentParameters[CP_SECCTRL0] = (((1 << 8) | (
  1 << 7)) | (1 << 6)) | (
  1 << 2);



  CC2420ControlM$gCurrentParameters[CP_IOCFG0] = (127 << 0) | (
  1 << 9);

  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 169
    CC2420ControlM$state = CC2420ControlM$INIT_STATE_DONE;
#line 169
    __nesc_atomic_end(__nesc_atomic); }
  CC2420ControlM$CmdSplitControlInit$release();
  CC2420ControlM$SplitControl$initDone();
}

# 75 "/Users/jingyuancheng/tinyos/moteiv/tos/lib/resource/ResourceCmd.nc"
inline static void CC2420ControlM$CmdSplitControlStart$release(void ){
#line 75
  /*MSP430ArbiterUSART0C.ArbiterC.ArbiterP*/FcfsArbiterP$1$ResourceCmd$release(/*CC2420RadioC.CmdSplitControlStartC.ResourceC.ResourceUSARTP*/MSP430ResourceUSART0P$2$ID);
#line 75
}
#line 75
# 414 "/Users/jingyuancheng/tinyos/moteiv/tos/lib/CC2420Radio/CC2420ControlM.nc"
static inline result_t CC2420ControlM$CC2420Control$OscillatorOn(uint8_t rh)
#line 414
{
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 415
    CC2420ControlM$cmds.oscillator = CC2420ControlM$CMD_OSCILLATOR_ON;
#line 415
    __nesc_atomic_end(__nesc_atomic); }
  CC2420ControlM$doCmds(rh);
  return SUCCESS;
}

# 34 "/Users/jingyuancheng/tinyos/moteiv/tos/platform/tmote/hardware.h"
static inline void TOSH_SET_CC_RSTN_PIN()
#line 34
{
#line 34
  static volatile uint8_t r __asm ("0x001D");

#line 34
  r |= 1 << 6;
}

#line 34
static inline void TOSH_CLR_CC_RSTN_PIN()
#line 34
{
#line 34
  static volatile uint8_t r __asm ("0x001D");

#line 34
  r &= ~(1 << 6);
}

#line 33
static inline void TOSH_SET_CC_VREN_PIN()
#line 33
{
#line 33
  static volatile uint8_t r __asm ("0x001D");

#line 33
  r |= 1 << 5;
}

# 430 "/Users/jingyuancheng/tinyos/moteiv/tos/lib/CC2420Radio/CC2420ControlM.nc"
static inline result_t CC2420ControlM$CC2420Control$VREFOn(void )
#line 430
{
  TOSH_SET_CC_VREN_PIN();

  TOSH_uwait(600);
  return SUCCESS;
}

# 99 "/Users/jingyuancheng/tinyos/moteiv/tos/platform/tmote/HPLCC2420M.nc"
static inline result_t HPLCC2420M$StdControl$start(void )
#line 99
{
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 100
    {
      if (HPLCC2420M$f_busy == HPLCC2420M$IDLE) {
          TOSH_SET_RADIO_CSN_PIN();
          TOSH_MAKE_RADIO_CSN_OUTPUT();
          HPLCC2420M$f_enabled = TRUE;
        }
    }
#line 106
    __nesc_atomic_end(__nesc_atomic); }
  return SUCCESS;
}

# 70 "/Users/jingyuancheng/tinyos/moteiv/tinyos-1.x/tos/interfaces/StdControl.nc"
inline static result_t CC2420ControlM$HPLChipconControl$start(void ){
#line 70
  unsigned char __nesc_result;
#line 70

#line 70
  __nesc_result = HPLCC2420M$StdControl$start();
#line 70

#line 70
  return __nesc_result;
#line 70
}
#line 70
# 230 "/Users/jingyuancheng/tinyos/moteiv/tos/lib/CC2420Radio/CC2420ControlM.nc"
static inline void CC2420ControlM$CmdSplitControlStart$granted(uint8_t rh)
#line 230
{
  CC2420ControlM$HPLChipconControl$start();

  CC2420ControlM$CC2420Control$VREFOn();

  TOSH_CLR_CC_RSTN_PIN();
  TOSH_wait();
  TOSH_SET_CC_RSTN_PIN();
  TOSH_wait();


  CC2420ControlM$CC2420Control$OscillatorOn(rh);
  CC2420ControlM$CmdSplitControlStart$release();
}

# 185 "/Users/jingyuancheng/tinyos/moteiv/tos/lib/sp/cc2420/CC2420AlwaysOnM.nc"
static inline result_t CC2420AlwaysOnM$RadioControl$stopDone(void )
#line 185
{
#line 185
  return SUCCESS;
}

# 99 "/Users/jingyuancheng/tinyos/moteiv/tinyos-1.x/tos/interfaces/SplitControl.nc"
inline static result_t CC2420RadioM$SplitControl$stopDone(void ){
#line 99
  unsigned char __nesc_result;
#line 99

#line 99
  __nesc_result = CC2420AlwaysOnM$RadioControl$stopDone();
#line 99

#line 99
  return __nesc_result;
#line 99
}
#line 99
# 138 "/Users/jingyuancheng/tinyos/moteiv/tos/lib/CC2420Radio/CC2420RadioM.nc"
static inline void CC2420RadioM$RadioActiveTime$default$overflow(void )
#line 138
{
}

# 70 "/Users/jingyuancheng/tinyos/moteiv/tos/lib/timer/Counter.nc"
inline static void CC2420RadioM$RadioActiveTime$overflow(void ){
#line 70
  CC2420RadioM$RadioActiveTime$default$overflow();
#line 70
}
#line 70
# 332 "/Users/jingyuancheng/tinyos/moteiv/tos/lib/CC2420Radio/CC2420RadioM.nc"
static inline result_t CC2420RadioM$CC2420SplitControl$stopDone(void )
#line 332
{
  CC2420RadioM$bShutdownRequest = FALSE;
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 334
    {
      uint32_t oldtime = CC2420RadioM$cc2420_waketime;

#line 336
      CC2420RadioM$cc2420_waketime += CC2420RadioM$Counter32khz$get() - CC2420RadioM$cc2420_laston;
      if (CC2420RadioM$cc2420_waketime < oldtime) {
        CC2420RadioM$RadioActiveTime$overflow();
        }
    }
#line 340
    __nesc_atomic_end(__nesc_atomic); }
#line 340
  return CC2420RadioM$SplitControl$stopDone();
}

# 99 "/Users/jingyuancheng/tinyos/moteiv/tinyos-1.x/tos/interfaces/SplitControl.nc"
inline static result_t CC2420ControlM$SplitControl$stopDone(void ){
#line 99
  unsigned char __nesc_result;
#line 99

#line 99
  __nesc_result = CC2420RadioM$CC2420SplitControl$stopDone();
#line 99

#line 99
  return __nesc_result;
#line 99
}
#line 99
# 75 "/Users/jingyuancheng/tinyos/moteiv/tos/lib/resource/ResourceCmd.nc"
inline static void CC2420ControlM$CmdSplitControlStop$release(void ){
#line 75
  /*MSP430ArbiterUSART0C.ArbiterC.ArbiterP*/FcfsArbiterP$1$ResourceCmd$release(/*CC2420RadioC.CmdSplitControlStopC.ResourceC.ResourceUSARTP*/MSP430ResourceUSART0P$3$ID);
#line 75
}
#line 75
# 33 "/Users/jingyuancheng/tinyos/moteiv/tos/platform/tmote/hardware.h"
static inline void TOSH_CLR_CC_VREN_PIN()
#line 33
{
#line 33
  static volatile uint8_t r __asm ("0x001D");

#line 33
  r &= ~(1 << 5);
}

# 437 "/Users/jingyuancheng/tinyos/moteiv/tos/lib/CC2420Radio/CC2420ControlM.nc"
static inline result_t CC2420ControlM$CC2420Control$VREFOff(void )
#line 437
{
  TOSH_CLR_CC_VREN_PIN();
  return SUCCESS;
}

# 110 "/Users/jingyuancheng/tinyos/moteiv/tos/platform/tmote/HPLCC2420M.nc"
static inline result_t HPLCC2420M$StdControl$stop(void )
#line 110
{
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 111
    HPLCC2420M$f_enabled = FALSE;
#line 111
    __nesc_atomic_end(__nesc_atomic); }
  return SUCCESS;
}

# 78 "/Users/jingyuancheng/tinyos/moteiv/tinyos-1.x/tos/interfaces/StdControl.nc"
inline static result_t CC2420ControlM$HPLChipconControl$stop(void ){
#line 78
  unsigned char __nesc_result;
#line 78

#line 78
  __nesc_result = HPLCC2420M$StdControl$stop();
#line 78

#line 78
  return __nesc_result;
#line 78
}
#line 78
# 168 "/Users/jingyuancheng/tinyos/moteiv/tos/platform/tmote/HPLCC2420InterruptM.nc"
static inline result_t HPLCC2420InterruptM$CCA$disable(void )
#line 168
{
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 169
    {
      HPLCC2420InterruptM$CCAInterrupt$disable();
      HPLCC2420InterruptM$CCAInterrupt$clear();
    }
#line 172
    __nesc_atomic_end(__nesc_atomic); }
  return SUCCESS;
}

# 59 "/Users/jingyuancheng/tinyos/moteiv/tinyos-1.x/tos/lib/CC2420Radio/HPLCC2420Interrupt.nc"
inline static result_t CC2420ControlM$CCA$disable(void ){
#line 59
  unsigned char __nesc_result;
#line 59

#line 59
  __nesc_result = HPLCC2420InterruptM$CCA$disable();
#line 59

#line 59
  return __nesc_result;
#line 59
}
#line 59
# 192 "/Users/jingyuancheng/tinyos/moteiv/tos/lib/CC2420Radio/CC2420ControlM.nc"
static inline void CC2420ControlM$CmdSplitControlStop$granted(uint8_t rh)
#line 192
{
  CC2420ControlM$HPLChipcon$cmd(rh, 0x07);
  CC2420ControlM$CCA$disable();
  CC2420ControlM$HPLChipconControl$stop();

  TOSH_CLR_CC_RSTN_PIN();
  CC2420ControlM$CC2420Control$VREFOff();
  TOSH_SET_CC_RSTN_PIN();

  CC2420ControlM$CmdSplitControlStop$release();
  CC2420ControlM$SplitControl$stopDone();
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 203
    CC2420ControlM$state = CC2420ControlM$INIT_STATE_DONE;
#line 203
    __nesc_atomic_end(__nesc_atomic); }
}

# 49 "/Users/jingyuancheng/tinyos/moteiv/tos/lib/sched/TaskBasic.nc"
inline static result_t HPLCC2420M$signalRXFIFO$postTask(void ){
#line 49
  unsigned char __nesc_result;
#line 49

#line 49
  __nesc_result = SchedulerBasicP$TaskBasic$postTask(HPLCC2420M$signalRXFIFO);
#line 49

#line 49
  return __nesc_result;
#line 49
}
#line 49
# 209 "/Users/jingyuancheng/tinyos/moteiv/tinyos-1.x/tos/platform/msp430/HPLUSARTControl.nc"
inline static uint8_t HPLCC2420M$USARTControl$rx(void ){
#line 209
  unsigned char __nesc_result;
#line 209

#line 209
  __nesc_result = HPLUSART0M$USARTControl$rx();
#line 209

#line 209
  return __nesc_result;
#line 209
}
#line 209
#line 185
inline static result_t HPLCC2420M$USARTControl$isRxIntrPending(void ){
#line 185
  unsigned char __nesc_result;
#line 185

#line 185
  __nesc_result = HPLUSART0M$USARTControl$isRxIntrPending();
#line 185

#line 185
  return __nesc_result;
#line 185
}
#line 185
# 460 "/Users/jingyuancheng/tinyos/moteiv/tos/platform/msp430/HPLUSART0M.nc"
static inline result_t HPLUSART0M$USARTControl$tx(uint8_t data)
#line 460
{
  HPLUSART0M$U0TXBUF = data;
  return SUCCESS;
}

# 202 "/Users/jingyuancheng/tinyos/moteiv/tinyos-1.x/tos/platform/msp430/HPLUSARTControl.nc"
inline static result_t HPLCC2420M$USARTControl$tx(uint8_t data){
#line 202
  unsigned char __nesc_result;
#line 202

#line 202
  __nesc_result = HPLUSART0M$USARTControl$tx(data);
#line 202

#line 202
  return __nesc_result;
#line 202
}
#line 202
#line 180
inline static result_t HPLCC2420M$USARTControl$isTxIntrPending(void ){
#line 180
  unsigned char __nesc_result;
#line 180

#line 180
  __nesc_result = HPLUSART0M$USARTControl$isTxIntrPending();
#line 180

#line 180
  return __nesc_result;
#line 180
}
#line 180
# 20 "/Users/jingyuancheng/tinyos/moteiv/tos/platform/tmote/hardware.h"
static inline void TOSH_CLR_RADIO_CSN_PIN()
#line 20
{
#line 20
  static volatile uint8_t r __asm ("0x001D");

#line 20
  r &= ~(1 << 2);
}

# 294 "/Users/jingyuancheng/tinyos/moteiv/tos/platform/tmote/HPLCC2420M.nc"
static inline result_t HPLCC2420M$HPLCC2420FIFO$readRXFIFO(uint8_t rh, uint8_t length, uint8_t *data)
#line 294
{
  uint8_t i;

#line 296
  if (HPLCC2420M$request(rh, HPLCC2420M$BUSY_RX)) {
      { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 297
        {
          HPLCC2420M$rxbuf = data;
          TOSH_CLR_RADIO_CSN_PIN();

          HPLCC2420M$USARTControl$isTxIntrPending();
          HPLCC2420M$USARTControl$rx();
          HPLCC2420M$USARTControl$tx(0x3F | 0x40);
          while (HPLCC2420M$f_enabled && !HPLCC2420M$USARTControl$isRxIntrPending()) ;
          HPLCC2420M$rxlen = HPLCC2420M$USARTControl$rx();
          HPLCC2420M$USARTControl$tx(0);
          while (HPLCC2420M$f_enabled && !HPLCC2420M$USARTControl$isRxIntrPending()) ;

          HPLCC2420M$rxlen = HPLCC2420M$USARTControl$rx();
        }
#line 310
        __nesc_atomic_end(__nesc_atomic); }
      if (HPLCC2420M$rxlen > 0) {
          HPLCC2420M$rxbuf[0] = HPLCC2420M$rxlen;

          HPLCC2420M$rxlen++;

          if (HPLCC2420M$rxlen > length) {
#line 316
            HPLCC2420M$rxlen = length;
            }
#line 317
          for (i = 1; i < HPLCC2420M$rxlen; i++) {
              { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 318
                {
                  HPLCC2420M$USARTControl$tx(0);
                  while (HPLCC2420M$f_enabled && !HPLCC2420M$USARTControl$isRxIntrPending()) ;
                  HPLCC2420M$rxbuf[i] = HPLCC2420M$USARTControl$rx();
                }
#line 322
                __nesc_atomic_end(__nesc_atomic); }
            }
        }
      TOSH_SET_RADIO_CSN_PIN();
      HPLCC2420M$signalRXFIFO$postTask();
      return SUCCESS;
    }
  return FAIL;
}

# 30 "/Users/jingyuancheng/tinyos/moteiv/tos/lib/CC2420Radio/HPLCC2420FIFO.nc"
inline static result_t CC2420RadioM$HPLChipconFIFO$readRXFIFO(uint8_t rh, uint8_t length, uint8_t *data){
#line 30
  unsigned char __nesc_result;
#line 30

#line 30
  __nesc_result = HPLCC2420M$HPLCC2420FIFO$readRXFIFO(rh, length, data);
#line 30

#line 30
  return __nesc_result;
#line 30
}
#line 30
# 722 "/Users/jingyuancheng/tinyos/moteiv/tos/lib/CC2420Radio/CC2420RadioM.nc"
static inline bool CC2420RadioM$delayedRXFIFOBody(uint8_t rh)
#line 722
{
  bool doReadRXFIFO;

  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 725
    {
      if (!TOSH_READ_CC_FIFO_PIN() && !TOSH_READ_CC_FIFOP_PIN()) {
          CC2420RadioM$flushRXFIFO(rh);
          {
            unsigned char __nesc_temp = 
#line 728
            FALSE;

            {
#line 728
              __nesc_atomic_end(__nesc_atomic); 
#line 728
              return __nesc_temp;
            }
          }
        }



      if (CC2420RadioM$bPacketReceiving == FALSE) {
          CC2420RadioM$bPacketReceiving = TRUE;
          CC2420RadioM$rh_receive = rh;
          doReadRXFIFO = TRUE;
        }
      else {
          doReadRXFIFO = FALSE;
        }
    }
#line 743
    __nesc_atomic_end(__nesc_atomic); }

  if (doReadRXFIFO) {
      if (CC2420RadioM$HPLChipconFIFO$readRXFIFO(rh, MSG_DATA_SIZE, (uint8_t *)CC2420RadioM$rxbufptr)) {
          return TRUE;
        }
      else {


          CC2420RadioM$flushRXFIFO(rh);
          return FALSE;
        }
    }
  else {




      return FALSE;
    }
}

static inline void CC2420RadioM$CmdReceive$granted(uint8_t rh)
#line 765
{
  if (CC2420RadioM$delayedRXFIFOBody(rh) == FALSE) {
    CC2420RadioM$CmdReceive$release();
    }
}

# 49 "/Users/jingyuancheng/tinyos/moteiv/tos/lib/sched/TaskBasic.nc"
inline static result_t HPLCC2420M$signalTXFIFO$postTask(void ){
#line 49
  unsigned char __nesc_result;
#line 49

#line 49
  __nesc_result = SchedulerBasicP$TaskBasic$postTask(HPLCC2420M$signalTXFIFO);
#line 49

#line 49
  return __nesc_result;
#line 49
}
#line 49
# 419 "/Users/jingyuancheng/tinyos/moteiv/tos/platform/msp430/HPLUSART0M.nc"
static inline result_t HPLUSART0M$USARTControl$isTxEmpty(void )
#line 419
{
  if (HPLUSART0M$U0TCTL & 0x01) {
      return SUCCESS;
    }
  return FAIL;
}

# 191 "/Users/jingyuancheng/tinyos/moteiv/tinyos-1.x/tos/platform/msp430/HPLUSARTControl.nc"
inline static result_t HPLCC2420M$USARTControl$isTxEmpty(void ){
#line 191
  unsigned char __nesc_result;
#line 191

#line 191
  __nesc_result = HPLUSART0M$USARTControl$isTxEmpty();
#line 191

#line 191
  return __nesc_result;
#line 191
}
#line 191
# 355 "/Users/jingyuancheng/tinyos/moteiv/tos/platform/tmote/HPLCC2420M.nc"
static inline result_t HPLCC2420M$HPLCC2420FIFO$writeTXFIFO(uint8_t rh, uint8_t length, uint8_t *data)
#line 355
{
  uint8_t i = 0;

#line 357
  if (HPLCC2420M$request(rh, HPLCC2420M$BUSY_TX)) {
      { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 358
        {
          HPLCC2420M$txlen = length;
          HPLCC2420M$txbuf = data;
        }
#line 361
        __nesc_atomic_end(__nesc_atomic); }
      TOSH_CLR_RADIO_CSN_PIN();

      HPLCC2420M$USARTControl$isTxIntrPending();
      HPLCC2420M$USARTControl$rx();
      HPLCC2420M$USARTControl$tx(0x3E);
      while (HPLCC2420M$f_enabled && !HPLCC2420M$USARTControl$isTxIntrPending()) ;
      for (i = 0; i < HPLCC2420M$txlen; i++) {
          HPLCC2420M$USARTControl$tx(HPLCC2420M$txbuf[i]);
          while (HPLCC2420M$f_enabled && !HPLCC2420M$USARTControl$isTxIntrPending()) ;
        }
      while (HPLCC2420M$f_enabled && !HPLCC2420M$USARTControl$isTxEmpty()) ;
      TOSH_SET_RADIO_CSN_PIN();
      HPLCC2420M$signalTXFIFO$postTask();
      return SUCCESS;
    }
  return FAIL;
}

# 42 "/Users/jingyuancheng/tinyos/moteiv/tos/lib/CC2420Radio/HPLCC2420FIFO.nc"
inline static result_t CC2420RadioM$HPLChipconFIFO$writeTXFIFO(uint8_t rh, uint8_t length, uint8_t *data){
#line 42
  unsigned char __nesc_result;
#line 42

#line 42
  __nesc_result = HPLCC2420M$HPLCC2420FIFO$writeTXFIFO(rh, length, data);
#line 42

#line 42
  return __nesc_result;
#line 42
}
#line 42
# 546 "/Users/jingyuancheng/tinyos/moteiv/tos/lib/CC2420Radio/CC2420RadioM.nc"
static inline bool CC2420RadioM$startSendBody(uint8_t rh)
#line 546
{
  if (CC2420RadioM$bShutdownRequest) {
      CC2420RadioM$sendFailedSync();
      return FALSE;
    }

  if (!CC2420RadioM$HPLChipcon$cmd(rh, 0x09)) {
      CC2420RadioM$sendFailedSync();
      return FALSE;
    }

  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 557
    CC2420RadioM$rh_transmit = rh;
#line 557
    __nesc_atomic_end(__nesc_atomic); }
  if (!CC2420RadioM$HPLChipconFIFO$writeTXFIFO(rh, CC2420RadioM$txlength + 1, (uint8_t *)CC2420RadioM$txbufptr)) {
      CC2420RadioM$sendFailedSync();
      return FALSE;
    }
  return TRUE;
}

static inline void CC2420RadioM$CmdTransmit$granted(uint8_t rh)
#line 565
{
  if (CC2420RadioM$startSendBody(rh) == FALSE) {
    CC2420RadioM$CmdTransmit$release();
    }
}

# 75 "/Users/jingyuancheng/tinyos/moteiv/tos/lib/resource/ResourceCmd.nc"
inline static void CC2420RadioM$CmdTryToSend$release(void ){
#line 75
  /*MSP430ArbiterUSART0C.ArbiterC.ArbiterP*/FcfsArbiterP$1$ResourceCmd$release(/*CC2420RadioC.CmdTryToSendC.ResourceC.ResourceUSARTP*/MSP430ResourceUSART0P$8$ID);
#line 75
}
#line 75
# 613 "/Users/jingyuancheng/tinyos/moteiv/tos/lib/CC2420Radio/CC2420RadioM.nc"
static inline void CC2420RadioM$CmdTryToSend$granted(uint8_t rh)
#line 613
{
  CC2420RadioM$tryToSend(rh);
  CC2420RadioM$CmdTryToSend$release();
}

# 233 "/Users/jingyuancheng/tinyos/moteiv/tos/lib/resource/FcfsArbiterP.nc"
static inline void /*MSP430ArbiterUSART0C.ArbiterC.ArbiterP*/FcfsArbiterP$1$ResourceCmd$default$granted(uint8_t id, uint8_t rh)
#line 233
{
}

# 70 "/Users/jingyuancheng/tinyos/moteiv/tos/lib/resource/ResourceCmd.nc"
inline static void /*MSP430ArbiterUSART0C.ArbiterC.ArbiterP*/FcfsArbiterP$1$ResourceCmd$granted(uint8_t arg_0x101b3b660, uint8_t rh){
#line 70
  switch (arg_0x101b3b660) {
#line 70
    case /*CC2420RadioC.CmdCCAFiredC.ResourceC.ResourceUSARTP*/MSP430ResourceUSART0P$0$ID:
#line 70
      CC2420ControlM$CmdCCAFired$granted(rh);
#line 70
      break;
#line 70
    case /*CC2420RadioC.CmdSplitControlInitC.ResourceC.ResourceUSARTP*/MSP430ResourceUSART0P$1$ID:
#line 70
      CC2420ControlM$CmdSplitControlInit$granted(rh);
#line 70
      break;
#line 70
    case /*CC2420RadioC.CmdSplitControlStartC.ResourceC.ResourceUSARTP*/MSP430ResourceUSART0P$2$ID:
#line 70
      CC2420ControlM$CmdSplitControlStart$granted(rh);
#line 70
      break;
#line 70
    case /*CC2420RadioC.CmdSplitControlStopC.ResourceC.ResourceUSARTP*/MSP430ResourceUSART0P$3$ID:
#line 70
      CC2420ControlM$CmdSplitControlStop$granted(rh);
#line 70
      break;
#line 70
    case /*CC2420RadioC.CmdReceiveC.ResourceC.ResourceUSARTP*/MSP430ResourceUSART0P$6$ID:
#line 70
      CC2420RadioM$CmdReceive$granted(rh);
#line 70
      break;
#line 70
    case /*CC2420RadioC.CmdTransmitC.ResourceC.ResourceUSARTP*/MSP430ResourceUSART0P$7$ID:
#line 70
      CC2420RadioM$CmdTransmit$granted(rh);
#line 70
      break;
#line 70
    case /*CC2420RadioC.CmdTryToSendC.ResourceC.ResourceUSARTP*/MSP430ResourceUSART0P$8$ID:
#line 70
      CC2420RadioM$CmdTryToSend$granted(rh);
#line 70
      break;
#line 70
    default:
#line 70
      /*MSP430ArbiterUSART0C.ArbiterC.ArbiterP*/FcfsArbiterP$1$ResourceCmd$default$granted(arg_0x101b3b660, rh);
#line 70
      break;
#line 70
    }
#line 70
}
#line 70
# 236 "/Users/jingyuancheng/tinyos/moteiv/tos/lib/resource/FcfsArbiterP.nc"
static inline void /*MSP430ArbiterUSART0C.ArbiterC.ArbiterP*/FcfsArbiterP$1$ResourceCmdAsync$default$granted(uint8_t id, uint8_t rh)
#line 236
{
}

# 80 "/Users/jingyuancheng/tinyos/moteiv/tos/lib/resource/ResourceCmdAsync.nc"
inline static void /*MSP430ArbiterUSART0C.ArbiterC.ArbiterP*/FcfsArbiterP$1$ResourceCmdAsync$granted(uint8_t arg_0x101b3a698, uint8_t rh){
#line 80
  switch (arg_0x101b3a698) {
#line 80
    case /*CC2420RadioC.CmdCmds.ResourceC.ResourceUSARTP*/MSP430ResourceUSART0P$4$ID:
#line 80
      CC2420ControlM$CmdCmds$granted(rh);
#line 80
      break;
#line 80
    case /*CC2420RadioC.CmdFlushRXFIFOC.ResourceC.ResourceUSARTP*/MSP430ResourceUSART0P$5$ID:
#line 80
      CC2420RadioM$CmdFlushRXFIFO$granted(rh);
#line 80
      break;
#line 80
    case /*CC2420TimeStampingC.CmdWriteTimeStampC.ResourceC.ResourceUSARTP*/MSP430ResourceUSART0P$9$ID:
#line 80
      CC2420TimeStampingM$CmdWriteTimeStamp$granted(rh);
#line 80
      break;
#line 80
    default:
#line 80
      /*MSP430ArbiterUSART0C.ArbiterC.ArbiterP*/FcfsArbiterP$1$ResourceCmdAsync$default$granted(arg_0x101b3a698, rh);
#line 80
      break;
#line 80
    }
#line 80
}
#line 80
# 57 "/Users/jingyuancheng/tinyos/moteiv/tos/lib/resource/FcfsArbiterP.nc"
static inline void /*MSP430ArbiterUSART0C.ArbiterC.ArbiterP*/FcfsArbiterP$1$GrantTask$runTask(void )
#line 57
{
  uint8_t id;

#line 59
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 59
    id = /*MSP430ArbiterUSART0C.ArbiterC.ArbiterP*/FcfsArbiterP$1$m_granted;
#line 59
    __nesc_atomic_end(__nesc_atomic); }
  if (id != RESOURCE_NONE) {
      { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 61
        /*MSP430ArbiterUSART0C.ArbiterC.ArbiterP*/FcfsArbiterP$1$ResourceConfigure$configure(id);
#line 61
        __nesc_atomic_end(__nesc_atomic); }
      /*MSP430ArbiterUSART0C.ArbiterC.ArbiterP*/FcfsArbiterP$1$ResourceCmdAsync$granted(id, id);
      /*MSP430ArbiterUSART0C.ArbiterC.ArbiterP*/FcfsArbiterP$1$ResourceCmd$granted(id, id);
      /*MSP430ArbiterUSART0C.ArbiterC.ArbiterP*/FcfsArbiterP$1$Resource$granted(id);
    }
}

# 201 "/Users/jingyuancheng/tinyos/moteiv/tos/lib/sched/SchedulerBasicP.nc"
static inline void SchedulerBasicP$TaskBasic$default$runTask(uint8_t id)
#line 201
{
}

# 58 "/Users/jingyuancheng/tinyos/moteiv/tos/lib/sched/TaskBasic.nc"
inline static void SchedulerBasicP$TaskBasic$runTask(uint8_t arg_0x101550368){
#line 58
  switch (arg_0x101550368) {
#line 58
    case 0U:
#line 58
      /*MSP430ArbiterTimerAC.ArbiterC.ArbiterP*/FcfsArbiterP$0$GrantTask$runTask();
#line 58
      break;
#line 58
    case ADC8IO14P$taskSendData:
#line 58
      ADC8IO14P$taskSendData$runTask();
#line 58
      break;
#line 58
    case /*HalTimerMilliC.AlarmToTimerC*/AlarmToTimerC$0$fired:
#line 58
      /*HalTimerMilliC.AlarmToTimerC*/AlarmToTimerC$0$fired$runTask();
#line 58
      break;
#line 58
    case /*HalTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC$0$executeTimersNow:
#line 58
      /*HalTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC$0$executeTimersNow$runTask();
#line 58
      break;
#line 58
    case FramerP$PacketSent:
#line 58
      FramerP$PacketSent$runTask();
#line 58
      break;
#line 58
    case FramerP$PacketUnknown:
#line 58
      FramerP$PacketUnknown$runTask();
#line 58
      break;
#line 58
    case FramerP$PacketRcvd:
#line 58
      FramerP$PacketRcvd$runTask();
#line 58
      break;
#line 58
    case UartPresenceM$taskConnected:
#line 58
      UartPresenceM$taskConnected$runTask();
#line 58
      break;
#line 58
    case UartPresenceM$taskDisconnected:
#line 58
      UartPresenceM$taskDisconnected$runTask();
#line 58
      break;
#line 58
    case FramerAckM$SendAckTask:
#line 58
      FramerAckM$SendAckTask$runTask();
#line 58
      break;
#line 58
    case CC2420RadioM$sendFailedTask:
#line 58
      CC2420RadioM$sendFailedTask$runTask();
#line 58
      break;
#line 58
    case CC2420RadioM$PacketRcvd:
#line 58
      CC2420RadioM$PacketRcvd$runTask();
#line 58
      break;
#line 58
    case CC2420RadioM$PacketSent:
#line 58
      CC2420RadioM$PacketSent$runTask();
#line 58
      break;
#line 58
    case CC2420RadioM$taskShutdownRequest:
#line 58
      CC2420RadioM$taskShutdownRequest$runTask();
#line 58
      break;
#line 58
    case CC2420RadioM$startRadio:
#line 58
      CC2420RadioM$startRadio$runTask();
#line 58
      break;
#line 58
    case HPLCC2420M$signalRAMRd:
#line 58
      HPLCC2420M$signalRAMRd$runTask();
#line 58
      break;
#line 58
    case HPLCC2420M$signalRAMWr:
#line 58
      HPLCC2420M$signalRAMWr$runTask();
#line 58
      break;
#line 58
    case HPLCC2420M$signalRXFIFO:
#line 58
      HPLCC2420M$signalRXFIFO$runTask();
#line 58
      break;
#line 58
    case HPLCC2420M$signalTXFIFO:
#line 58
      HPLCC2420M$signalTXFIFO$runTask();
#line 58
      break;
#line 58
    case 19U:
#line 58
      /*MSP430ArbiterUSART0C.ArbiterC.ArbiterP*/FcfsArbiterP$1$GrantTask$runTask();
#line 58
      break;
#line 58
    case RefVoltM$switchOnDelay:
#line 58
      RefVoltM$switchOnDelay$runTask();
#line 58
      break;
#line 58
    case RefVoltM$switchOffDelay:
#line 58
      RefVoltM$switchOffDelay$runTask();
#line 58
      break;
#line 58
    case RefVoltM$switchOffRetry:
#line 58
      RefVoltM$switchOffRetry$runTask();
#line 58
      break;
#line 58
    default:
#line 58
      SchedulerBasicP$TaskBasic$default$runTask(arg_0x101550368);
#line 58
      break;
#line 58
    }
#line 58
}
#line 58
# 28 "/Users/jingyuancheng/tinyos/moteiv/tos/platform/tmote/hardware.h"
static inline uint8_t TOSH_READ_RADIO_CCA_PIN()
#line 28
{
#line 28
  static volatile uint8_t r __asm ("0x0020");

#line 28
  return r & (1 << 4);
}

# 198 "/Users/jingyuancheng/tinyos/moteiv/tos/lib/CC2420Radio/CC2420RadioM.nc"
static __inline result_t CC2420RadioM$setBackoffTimer(uint16_t jiffy)
#line 198
{
  CC2420RadioM$stateTimer = CC2420RadioM$TIMER_BACKOFF;
  CC2420RadioM$BackoffAlarm32khz$start(jiffy);
  return SUCCESS;
}







static __inline result_t CC2420RadioM$setSFDTimeoutTimer(uint16_t jiffy)
#line 210
{
  CC2420RadioM$stateTimer = CC2420RadioM$TIMER_SFD;
  CC2420RadioM$BackoffAlarm32khz$start(jiffy);
  return SUCCESS;
}

#line 432
static inline void CC2420RadioM$sendPacket(uint8_t rh)
#line 432
{
  uint8_t status;

  CC2420RadioM$HPLChipcon$cmd(rh, 0x05);
  status = CC2420RadioM$HPLChipcon$cmd(rh, 0x00);
  if ((status >> 3) & 0x01) {


      CC2420RadioM$SFD$enableCapture(TRUE);

      CC2420RadioM$setSFDTimeoutTimer(150);
    }
  else {

      { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 446
        CC2420RadioM$stateRadio = CC2420RadioM$PRE_TX_STATE;
#line 446
        __nesc_atomic_end(__nesc_atomic); }
      if (!CC2420RadioM$setBackoffTimer(CC2420RadioM$MacBackoff$congestionBackoff(CC2420RadioM$txbufptr) * 2)) {
          CC2420RadioM$sendFailedAsync();
        }
    }
}

# 311 "/Users/jingyuancheng/tinyos/moteiv/tos/lib/sp/SPM.nc"
static inline void SPM$LowerSend$sendDone(sp_message_t *msg, sp_message_flags_t flags, sp_error_t success)
#line 311
{

  if (SPM$m_sending && SPM$m_currentmsg == msg) {
      SPM$processSendComplete(msg, flags, success);
    }


  SPM$nextSend();
}

# 126 "/Users/jingyuancheng/tinyos/moteiv/tos/lib/sp/SPSend.nc"
inline static void CC2420AlwaysOnM$SPSend$sendDone(sp_message_t *msg, sp_message_flags_t flags, sp_error_t error){
#line 126
  SPM$LowerSend$sendDone(msg, flags, error);
#line 126
}
#line 126
# 97 "/Users/jingyuancheng/tinyos/moteiv/tos/lib/util/pool/ObjectPoolC.nc"
static inline uint8_t /*SPC.NeighborTable*/ObjectPoolC$1$Pool$first(void )
#line 97
{
  return /*SPC.NeighborTable*/ObjectPoolC$1$m_pool[0] != NULL ? 0 : /*SPC.NeighborTable*/ObjectPoolC$1$Pool$next(0);
}

# 69 "/Users/jingyuancheng/tinyos/moteiv/tos/lib/util/pool/ObjectPool.nc"
inline static uint8_t SPNeighborTableM$NeighborTable$first(void ){
#line 69
  unsigned char __nesc_result;
#line 69

#line 69
  __nesc_result = /*SPC.NeighborTable*/ObjectPoolC$1$Pool$first();
#line 69

#line 69
  return __nesc_result;
#line 69
}
#line 69
# 36 "/Users/jingyuancheng/tinyos/moteiv/tos/lib/sp/SPNeighborTableM.nc"
static inline uint8_t SPNeighborTableM$SPNeighbor$first(uint8_t id)
#line 36
{
  return SPNeighborTableM$NeighborTable$first();
}

# 29 "/Users/jingyuancheng/tinyos/moteiv/tos/lib/sp/SPNeighbor.nc"
inline static uint8_t SPM$SPNeighbor$first(void ){
#line 29
  unsigned char __nesc_result;
#line 29

#line 29
  __nesc_result = SPNeighborTableM$SPNeighbor$first(0U);
#line 29

#line 29
  return __nesc_result;
#line 29
}
#line 29
# 101 "/Users/jingyuancheng/tinyos/moteiv/tos/lib/util/pool/ObjectPoolC.nc"
static inline bool /*SPC.NeighborTable*/ObjectPoolC$1$Pool$valid(uint8_t n)
#line 101
{
  return n < 10;
}

# 71 "/Users/jingyuancheng/tinyos/moteiv/tos/lib/util/pool/ObjectPool.nc"
inline static bool SPNeighborTableM$NeighborTable$valid(uint8_t n){
#line 71
  unsigned char __nesc_result;
#line 71

#line 71
  __nesc_result = /*SPC.NeighborTable*/ObjectPoolC$1$Pool$valid(n);
#line 71

#line 71
  return __nesc_result;
#line 71
}
#line 71
# 39 "/Users/jingyuancheng/tinyos/moteiv/tos/lib/sp/SPNeighborTableM.nc"
static inline bool SPNeighborTableM$SPNeighbor$valid(uint8_t id, uint8_t i)
#line 39
{
  return SPNeighborTableM$NeighborTable$valid(i);
}

# 30 "/Users/jingyuancheng/tinyos/moteiv/tos/lib/sp/SPNeighbor.nc"
inline static bool SPM$SPNeighbor$valid(uint8_t n){
#line 30
  unsigned char __nesc_result;
#line 30

#line 30
  __nesc_result = SPNeighborTableM$SPNeighbor$valid(0U, n);
#line 30

#line 30
  return __nesc_result;
#line 30
}
#line 30
# 72 "/Users/jingyuancheng/tinyos/moteiv/tos/lib/util/pool/ObjectPool.nc"
inline static uint8_t SPNeighborTableM$NeighborTable$next(uint8_t n){
#line 72
  unsigned char __nesc_result;
#line 72

#line 72
  __nesc_result = /*SPC.NeighborTable*/ObjectPoolC$1$Pool$next(n);
#line 72

#line 72
  return __nesc_result;
#line 72
}
#line 72
# 42 "/Users/jingyuancheng/tinyos/moteiv/tos/lib/sp/SPNeighborTableM.nc"
static inline uint8_t SPNeighborTableM$SPNeighbor$next(uint8_t id, uint8_t i)
#line 42
{
  return SPNeighborTableM$NeighborTable$next(i);
}

# 31 "/Users/jingyuancheng/tinyos/moteiv/tos/lib/sp/SPNeighbor.nc"
inline static uint8_t SPM$SPNeighbor$next(uint8_t n){
#line 31
  unsigned char __nesc_result;
#line 31

#line 31
  __nesc_result = SPNeighborTableM$SPNeighbor$next(0U, n);
#line 31

#line 31
  return __nesc_result;
#line 31
}
#line 31
# 93 "/Users/jingyuancheng/tinyos/moteiv/tos/lib/util/pool/ObjectPoolC.nc"
static inline /*SPC.NeighborTable*/ObjectPoolC$1$object_type */*SPC.NeighborTable*/ObjectPoolC$1$Pool$get(uint8_t n)
#line 93
{
  return /*SPC.NeighborTable*/ObjectPoolC$1$m_pool[n];
}

# 62 "/Users/jingyuancheng/tinyos/moteiv/tos/lib/util/pool/ObjectPool.nc"
inline static SPNeighborTableM$NeighborTable$object_type *SPNeighborTableM$NeighborTable$get(uint8_t position){
#line 62
  struct SPNeighborTableEntry *__nesc_result;
#line 62

#line 62
  __nesc_result = /*SPC.NeighborTable*/ObjectPoolC$1$Pool$get(position);
#line 62

#line 62
  return __nesc_result;
#line 62
}
#line 62
# 30 "/Users/jingyuancheng/tinyos/moteiv/tos/lib/sp/SPNeighborTableM.nc"
static inline sp_neighbor_t *SPNeighborTableM$SPNeighbor$get(uint8_t id, uint8_t i)
#line 30
{
  return SPNeighborTableM$NeighborTable$get(i);
}

# 22 "/Users/jingyuancheng/tinyos/moteiv/tos/lib/sp/SPNeighbor.nc"
inline static sp_neighbor_t *SPM$SPNeighbor$get(uint8_t n){
#line 22
  struct SPNeighborTableEntry *__nesc_result;
#line 22

#line 22
  __nesc_result = SPNeighborTableM$SPNeighbor$get(0U, n);
#line 22

#line 22
  return __nesc_result;
#line 22
}
#line 22
# 47 "/Users/jingyuancheng/tinyos/moteiv/tos/lib/util/pool/ObjectPool.nc"
inline static uint8_t SPNeighborTableM$NeighborTable$populated(void ){
#line 47
  unsigned char __nesc_result;
#line 47

#line 47
  __nesc_result = /*SPC.NeighborTable*/ObjectPoolC$1$Pool$populated();
#line 47

#line 47
  return __nesc_result;
#line 47
}
#line 47
# 45 "/Users/jingyuancheng/tinyos/moteiv/tos/lib/sp/SPNeighborTableM.nc"
static inline uint8_t SPNeighborTableM$SPNeighbor$populated(uint8_t id)
#line 45
{
  return SPNeighborTableM$NeighborTable$populated();
}

# 32 "/Users/jingyuancheng/tinyos/moteiv/tos/lib/sp/SPNeighbor.nc"
inline static uint8_t SPM$SPNeighbor$populated(void ){
#line 32
  unsigned char __nesc_result;
#line 32

#line 32
  __nesc_result = SPNeighborTableM$SPNeighbor$populated(0U);
#line 32

#line 32
  return __nesc_result;
#line 32
}
#line 32
# 62 "/Users/jingyuancheng/tinyos/moteiv/tos/lib/sp/SPM.nc"
static inline uint8_t SPM$neighborPopulation(void )
#line 62
{
  return SPM$SPNeighbor$populated();
}

# 152 "/Users/jingyuancheng/tinyos/moteiv/tos/lib/sp/cc2420/CC2420AlwaysOnM.nc"
static inline sp_linkstate_t CC2420AlwaysOnM$SPLinkStats$getState(void )
#line 152
{
  return SP_RADIO_ON;
}

# 23 "/Users/jingyuancheng/tinyos/moteiv/tos/lib/sp/SPLinkStats.nc"
inline static sp_linkstate_t SPM$SPLinkStats$getState(void ){
#line 23
  enum __nesc_unnamed4290 __nesc_result;
#line 23

#line 23
  __nesc_result = CC2420AlwaysOnM$SPLinkStats$getState();
#line 23

#line 23
  return __nesc_result;
#line 23
}
#line 23
# 90 "/Users/jingyuancheng/tinyos/moteiv/tos/lib/sp/SPSend.nc"
inline static result_t SPM$LowerSend$sendAdv(sp_message_t *msg, TOS_Msg *tosmsg, sp_device_t dev, sp_address_t addr, uint8_t length, sp_message_flags_t flags, uint8_t quantity){
#line 90
  unsigned char __nesc_result;
#line 90

#line 90
  __nesc_result = CC2420AlwaysOnM$SPSend$sendAdv(msg, tosmsg, dev, addr, length, flags, quantity);
#line 90

#line 90
  return __nesc_result;
#line 90
}
#line 90
# 100 "/Users/jingyuancheng/tinyos/moteiv/tos/lib/sp/cc2420/CC2420TimeStampingM.nc"
static inline result_t CC2420TimeStampingM$TimeStamping$addStamp(TOS_MsgPtr msg, int8_t offset)
{
  if (0 <= offset && offset <= 66 - 4) {
      { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 103
        CC2420TimeStampingM$sendStampOffset = offset;
#line 103
        __nesc_atomic_end(__nesc_atomic); }
      CC2420TimeStampingM$ptosMsg = msg;
      return SUCCESS;
    }
  return FAIL;
}

# 79 "/Users/jingyuancheng/tinyos/moteiv/tos/lib/sp/cc2420/TimeStamping.nc"
inline static result_t CC2420AlwaysOnM$TimeStamping$addStamp(TOS_MsgPtr msg, int8_t offset){
#line 79
  unsigned char __nesc_result;
#line 79

#line 79
  __nesc_result = CC2420TimeStampingM$TimeStamping$addStamp(msg, offset);
#line 79

#line 79
  return __nesc_result;
#line 79
}
#line 79
# 167 "/Users/jingyuancheng/tinyos/moteiv/tos/lib/sp/cc2420/CC2420AlwaysOnM.nc"
static inline int16_t CC2420AlwaysOnM$MacBackoff$initialBackoff(TOS_MsgPtr m)
#line 167
{
  if (CC2420AlwaysOnM$m_flags & CC2420AlwaysOnM$FLAG_MULTIMSG) {
    return (CC2420AlwaysOnM$Random$rand() & 0x1F) + 1;
    }
  else {
#line 171
    return (CC2420AlwaysOnM$Random$rand() & 0x7F) + 1;
    }
}

# 19 "/Users/jingyuancheng/tinyos/moteiv/tos/lib/CC2420Radio/MacBackoff.nc"
inline static int16_t CC2420RadioM$MacBackoff$initialBackoff(TOS_MsgPtr m){
#line 19
  int __nesc_result;
#line 19

#line 19
  __nesc_result = CC2420AlwaysOnM$MacBackoff$initialBackoff(m);
#line 19

#line 19
  return __nesc_result;
#line 19
}
#line 19
# 192 "/Users/jingyuancheng/tinyos/moteiv/tos/lib/CC2420Radio/CC2420RadioM.nc"
static __inline result_t CC2420RadioM$setInitialTimer(uint16_t jiffy)
#line 192
{
  CC2420RadioM$stateTimer = CC2420RadioM$TIMER_INITIAL;
  CC2420RadioM$BackoffAlarm32khz$start(jiffy);
  return SUCCESS;
}

#line 682
static inline result_t CC2420RadioM$Send$send(TOS_MsgPtr pMsg)
#line 682
{
  uint8_t currentstate;

#line 684
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 684
    currentstate = CC2420RadioM$stateRadio;
#line 684
    __nesc_atomic_end(__nesc_atomic); }

  if (currentstate == CC2420RadioM$IDLE_STATE && !CC2420RadioM$bShutdownRequest) {

      pMsg->fcflo = 0x08;
      pMsg->fcfhi = 0x01;

      pMsg->destpan = TOS_BCAST_ADDR;

      pMsg->addr = toLSB16(pMsg->addr);

      pMsg->length = pMsg->length + MSG_HEADER_SIZE + MSG_FOOTER_SIZE;

      pMsg->dsn = ++CC2420RadioM$currentDSN;

      pMsg->time = 0;

      CC2420RadioM$txlength = pMsg->length - MSG_FOOTER_SIZE;
      CC2420RadioM$txbufptr = pMsg;
      CC2420RadioM$countRetry = 8;

      if (CC2420RadioM$setInitialTimer(CC2420RadioM$MacBackoff$initialBackoff(CC2420RadioM$txbufptr) * 2)) {
          { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 706
            CC2420RadioM$stateRadio = CC2420RadioM$PRE_TX_STATE;
#line 706
            __nesc_atomic_end(__nesc_atomic); }
          return SUCCESS;
        }
    }
  return FAIL;
}

# 59 "/Users/jingyuancheng/tinyos/moteiv/tos/lib/CC2420Radio/CC2420BareSendMsg.nc"
inline static result_t CC2420AlwaysOnM$LowerSend$send(TOS_MsgPtr msg){
#line 59
  unsigned char __nesc_result;
#line 59

#line 59
  __nesc_result = CC2420RadioM$Send$send(msg);
#line 59

#line 59
  return __nesc_result;
#line 59
}
#line 59
# 664 "/Users/jingyuancheng/tinyos/moteiv/tos/lib/CC2420Radio/CC2420RadioM.nc"
static inline void CC2420RadioM$MacControl$requestAck(TOS_MsgPtr pMsg)
#line 664
{
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 665
    {
      if (pMsg == CC2420RadioM$txbufptr) {
          CC2420RadioM$txbufptr->fcfhi = 0x21;
        }
    }
#line 669
    __nesc_atomic_end(__nesc_atomic); }
}

# 31 "/Users/jingyuancheng/tinyos/moteiv/tos/lib/CC2420Radio/MacControl.nc"
inline static void CC2420AlwaysOnM$MacControl$requestAck(TOS_MsgPtr msg){
#line 31
  CC2420RadioM$MacControl$requestAck(msg);
#line 31
}
#line 31
# 33 "/Users/jingyuancheng/tinyos/moteiv/tos/lib/util/pool/ObjectPool.nc"
inline static result_t SPM$Pool$remove(SPM$Pool$object_type *obj){
#line 33
  unsigned char __nesc_result;
#line 33

#line 33
  __nesc_result = /*SPC.MessagePool*/ObjectPoolC$0$Pool$remove(obj);
#line 33

#line 33
  return __nesc_result;
#line 33
}
#line 33
# 149 "/Users/jingyuancheng/tinyos/moteiv/tos/lib/sp/SPM.nc"
static inline void SPM$PoolEvents$removed(sp_message_t *msg)
#line 149
{
}

# 26 "/Users/jingyuancheng/tinyos/moteiv/tos/lib/util/pool/ObjectPoolEvents.nc"
inline static void /*SPC.MessagePool*/ObjectPoolC$0$PoolEvents$removed(/*SPC.MessagePool*/ObjectPoolC$0$PoolEvents$object_type *object){
#line 26
  SPM$PoolEvents$removed(object);
#line 26
}
#line 26
# 29 "/Users/jingyuancheng/tinyos/moteiv/tos/lib/sp/SPAdaptorGenericCommM.nc"
static inline bool SPAdaptorGenericCommM$contains(sp_message_t *_msg)
#line 29
{










  return _msg >= SPAdaptorGenericCommM$m_pool + 0 && _msg < SPAdaptorGenericCommM$m_pool + 1;
}

# 70 "/Users/jingyuancheng/tinyos/moteiv/tinyos-1.x/tos/platform/msp430/MSP430GeneralIO.h"
static inline void TOSH_SET_PORT56_PIN()
#line 70
{
#line 70
  static volatile uint8_t r __asm ("0x0031");

#line 70
  r |= 1 << 6;
}

# 465 "/Users/jingyuancheng/tinyos/moteiv/tinyos-1.x/tos/platform/msp430/MSP430GeneralIOM.nc"
static inline void MSP430GeneralIOM$Port56$setHigh(void )
#line 465
{
#line 465
  TOSH_SET_PORT56_PIN();
}

# 27 "/Users/jingyuancheng/tinyos/moteiv/tinyos-1.x/tos/platform/msp430/MSP430GeneralIO.nc"
inline static void ADC8IO14P$LedUsr$setHigh(void ){
#line 27
  MSP430GeneralIOM$Port56$setHigh();
#line 27
}
#line 27
# 329 "ADC8IO14P.nc"
static inline result_t ADC8IO14P$SendMsg$sendDone(TOS_MsgPtr msg, result_t success)
{
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
    {
      ADC8IO14P$m_sending = FALSE;
    }
#line 334
    __nesc_atomic_end(__nesc_atomic); }
  ADC8IO14P$LedUsr$setHigh();
  return SUCCESS;
}

# 79 "/Users/jingyuancheng/tinyos/moteiv/tos/lib/sp/SPAdaptorGenericCommM.nc"
static inline result_t SPAdaptorGenericCommM$SendMsg$default$sendDone(uint8_t id, TOS_MsgPtr p, result_t s)
#line 79
{
#line 79
  return SUCCESS;
}

# 49 "/Users/jingyuancheng/tinyos/moteiv/tinyos-1.x/tos/interfaces/SendMsg.nc"
inline static result_t SPAdaptorGenericCommM$SendMsg$sendDone(uint8_t arg_0x102da0790, TOS_MsgPtr msg, result_t success){
#line 49
  unsigned char __nesc_result;
#line 49

#line 49
  switch (arg_0x102da0790) {
#line 49
    case 31:
#line 49
      __nesc_result = ADC8IO14P$SendMsg$sendDone(msg, success);
#line 49
      break;
#line 49
    default:
#line 49
      __nesc_result = SPAdaptorGenericCommM$SendMsg$default$sendDone(arg_0x102da0790, msg, success);
#line 49
      break;
#line 49
    }
#line 49

#line 49
  return __nesc_result;
#line 49
}
#line 49
# 49 "/Users/jingyuancheng/tinyos/moteiv/tos/lib/timer/LocalTime.nc"
inline static uint32_t SPM$LocalTime$get(void ){
#line 49
  unsigned long __nesc_result;
#line 49

#line 49
  __nesc_result = /*Counter32khzC.CounterToLocalTimeC*/CounterToLocalTimeC$1$LocalTime$get();
#line 49

#line 49
  return __nesc_result;
#line 49
}
#line 49
# 97 "/Users/jingyuancheng/tinyos/moteiv/tos/lib/util/pool/ObjectPoolC.nc"
static inline uint8_t /*SPC.MessagePool*/ObjectPoolC$0$Pool$first(void )
#line 97
{
  return /*SPC.MessagePool*/ObjectPoolC$0$m_pool[0] != NULL ? 0 : /*SPC.MessagePool*/ObjectPoolC$0$Pool$next(0);
}

# 69 "/Users/jingyuancheng/tinyos/moteiv/tos/lib/util/pool/ObjectPool.nc"
inline static uint8_t SPM$Pool$first(void ){
#line 69
  unsigned char __nesc_result;
#line 69

#line 69
  __nesc_result = /*SPC.MessagePool*/ObjectPoolC$0$Pool$first();
#line 69

#line 69
  return __nesc_result;
#line 69
}
#line 69
# 101 "/Users/jingyuancheng/tinyos/moteiv/tos/lib/util/pool/ObjectPoolC.nc"
static inline bool /*SPC.MessagePool*/ObjectPoolC$0$Pool$valid(uint8_t n)
#line 101
{
  return n < 10;
}

# 71 "/Users/jingyuancheng/tinyos/moteiv/tos/lib/util/pool/ObjectPool.nc"
inline static bool SPM$Pool$valid(uint8_t n){
#line 71
  unsigned char __nesc_result;
#line 71

#line 71
  __nesc_result = /*SPC.MessagePool*/ObjectPoolC$0$Pool$valid(n);
#line 71

#line 71
  return __nesc_result;
#line 71
}
#line 71

inline static uint8_t SPM$Pool$next(uint8_t n){
#line 72
  unsigned char __nesc_result;
#line 72

#line 72
  __nesc_result = /*SPC.MessagePool*/ObjectPoolC$0$Pool$next(n);
#line 72

#line 72
  return __nesc_result;
#line 72
}
#line 72
# 93 "/Users/jingyuancheng/tinyos/moteiv/tos/lib/util/pool/ObjectPoolC.nc"
static inline /*SPC.MessagePool*/ObjectPoolC$0$object_type */*SPC.MessagePool*/ObjectPoolC$0$Pool$get(uint8_t n)
#line 93
{
  return /*SPC.MessagePool*/ObjectPoolC$0$m_pool[n];
}

# 62 "/Users/jingyuancheng/tinyos/moteiv/tos/lib/util/pool/ObjectPool.nc"
inline static SPM$Pool$object_type *SPM$Pool$get(uint8_t position){
#line 62
  struct SPMessage *__nesc_result;
#line 62

#line 62
  __nesc_result = /*SPC.MessagePool*/ObjectPoolC$0$Pool$get(position);
#line 62

#line 62
  return __nesc_result;
#line 62
}
#line 62
# 48 "/Users/jingyuancheng/tinyos/moteiv/tos/platform/msp430/timer/MSP430AlarmC.nc"
static inline result_t /*CC2420RadioC.BackoffAlarm32khzC.MSP430Alarm*/MSP430AlarmC$1$Init$start(void )
#line 48
{
#line 48
  return SUCCESS;
}

# 70 "/Users/jingyuancheng/tinyos/moteiv/tinyos-1.x/tos/interfaces/StdControl.nc"
inline static result_t CC2420RadioM$TimerControl$start(void ){
#line 70
  unsigned char __nesc_result;
#line 70

#line 70
  __nesc_result = /*CC2420RadioC.BackoffAlarm32khzC.MSP430Alarm*/MSP430AlarmC$1$Init$start();
#line 70

#line 70
  return __nesc_result;
#line 70
}
#line 70
# 63 "/Users/jingyuancheng/tinyos/moteiv/tos/lib/resource/ResourceCmd.nc"
inline static void CC2420ControlM$CmdSplitControlStart$deferRequest(void ){
#line 63
  /*MSP430ArbiterUSART0C.ArbiterC.ArbiterP*/FcfsArbiterP$1$ResourceCmd$deferRequest(/*CC2420RadioC.CmdSplitControlStartC.ResourceC.ResourceUSARTP*/MSP430ResourceUSART0P$2$ID);
#line 63
}
#line 63
# 245 "/Users/jingyuancheng/tinyos/moteiv/tos/lib/CC2420Radio/CC2420ControlM.nc"
static inline result_t CC2420ControlM$SplitControl$start(void )
#line 245
{
  uint8_t _state = FALSE;

  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 248
    {
      if (CC2420ControlM$state == CC2420ControlM$INIT_STATE_DONE) {
          CC2420ControlM$state = CC2420ControlM$START_STATE;
          _state = TRUE;
        }
    }
#line 253
    __nesc_atomic_end(__nesc_atomic); }
  if (!_state) {
    return FAIL;
    }
  CC2420ControlM$CmdSplitControlStart$deferRequest();
  return SUCCESS;
}

# 77 "/Users/jingyuancheng/tinyos/moteiv/tinyos-1.x/tos/interfaces/SplitControl.nc"
inline static result_t CC2420RadioM$CC2420SplitControl$start(void ){
#line 77
  unsigned char __nesc_result;
#line 77

#line 77
  __nesc_result = CC2420ControlM$SplitControl$start();
#line 77

#line 77
  return __nesc_result;
#line 77
}
#line 77
# 172 "/Users/jingyuancheng/tinyos/moteiv/tos/lib/timer/VirtualizeTimerC.nc"
static inline void /*HalTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC$0$Timer$stop(uint8_t num)
{
  /*HalTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC$0$m_flags[num].isrunning = FALSE;
}

# 64 "/Users/jingyuancheng/tinyos/moteiv/tos/lib/timer/Timer2.nc"
inline static void TimerWrapC$Timer2$stop(uint8_t arg_0x1025b4258){
#line 64
  /*HalTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC$0$Timer$stop(arg_0x1025b4258);
#line 64
}
#line 64
# 41 "/Users/jingyuancheng/tinyos/moteiv/tos/lib/timer/TimerWrapC.nc"
static inline result_t TimerWrapC$Timer$stop(uint8_t id)
{
  TimerWrapC$Timer2$stop(id);
  return SUCCESS;
}

# 68 "/Users/jingyuancheng/tinyos/moteiv/tinyos-1.x/tos/interfaces/Timer.nc"
inline static result_t FramerP$Timer$stop(void ){
#line 68
  unsigned char __nesc_result;
#line 68

#line 68
  __nesc_result = TimerWrapC$Timer$stop(1U);
#line 68

#line 68
  return __nesc_result;
#line 68
}
#line 68
# 43 "/Users/jingyuancheng/tinyos/moteiv/tos/platform/tmote/hardware.h"
static inline void TOSH_SEL_UTXD1_MODFUNC()
#line 43
{
#line 43
  static volatile uint8_t r __asm ("0x001B");

#line 43
  r |= 1 << 6;
}

# 150 "/Users/jingyuancheng/tinyos/moteiv/tos/platform/msp430/HPLUSART1M.nc"
static inline void HPLUSART1M$USARTControl$enableUARTTx(void )
#line 150
{
  TOSH_SEL_UTXD1_MODFUNC();
  HPLUSART1M$ME2 |= 1 << 5;
}

# 90 "/Users/jingyuancheng/tinyos/moteiv/tinyos-1.x/tos/platform/msp430/HPLUSARTControl.nc"
inline static void FramerP$USARTControl$enableUARTTx(void ){
#line 90
  HPLUSART1M$USARTControl$enableUARTTx();
#line 90
}
#line 90
# 399 "/Users/jingyuancheng/tinyos/moteiv/tos/platform/msp430/HPLUSART1M.nc"
static inline result_t HPLUSART1M$USARTControl$tx(uint8_t data)
#line 399
{
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 400
    {
      HPLUSART1M$U1TXBUF = data;
    }
#line 402
    __nesc_atomic_end(__nesc_atomic); }
  return SUCCESS;
}

# 202 "/Users/jingyuancheng/tinyos/moteiv/tinyos-1.x/tos/platform/msp430/HPLUSARTControl.nc"
inline static result_t HPLUARTM$USARTControl$tx(uint8_t data){
#line 202
  unsigned char __nesc_result;
#line 202

#line 202
  __nesc_result = HPLUSART1M$USARTControl$tx(data);
#line 202

#line 202
  return __nesc_result;
#line 202
}
#line 202
# 98 "/Users/jingyuancheng/tinyos/moteiv/tinyos-1.x/tos/platform/msp430/HPLUARTM.nc"
static inline result_t HPLUARTM$UART$put(uint8_t data)
#line 98
{
  return HPLUARTM$USARTControl$tx(data);
}

# 80 "/Users/jingyuancheng/tinyos/moteiv/tinyos-1.x/tos/interfaces/HPLUART.nc"
inline static result_t UARTM$HPLUART$put(uint8_t data){
#line 80
  unsigned char __nesc_result;
#line 80

#line 80
  __nesc_result = HPLUARTM$UART$put(data);
#line 80

#line 80
  return __nesc_result;
#line 80
}
#line 80
# 339 "/Users/jingyuancheng/tinyos/moteiv/tos/platform/msp430/adc/MSP430ADC12M.nc"
static inline msp430ADCresult_t MSP430ADC12M$ADCSingle$getData(uint8_t num)
{
  return MSP430ADC12M$newRequest(SINGLE_CHANNEL, num, 0, 1, 0);
}

# 67 "/Users/jingyuancheng/tinyos/moteiv/tos/platform/msp430/adc/MSP430ADC12Single.nc"
inline static msp430ADCresult_t ADC8IO14P$ADC0$getData(void ){
#line 67
  enum __nesc_unnamed4259 __nesc_result;
#line 67

#line 67
  __nesc_result = MSP430ADC12M$ADCSingle$getData(0U);
#line 67

#line 67
  return __nesc_result;
#line 67
}
#line 67
# 209 "ADC8IO14P.nc"
static inline void ADC8IO14P$Timer$fired(void )
{



  ADC8IO14P$switchCtrl();
  ADC8IO14P$ADC0$getData();
}

# 178 "/Users/jingyuancheng/tinyos/moteiv/tos/lib/sp/cc2420/CC2420AlwaysOnM.nc"
static inline void CC2420AlwaysOnM$SanityTimer$fired(void )
#line 178
{
}

# 127 "/Users/jingyuancheng/tinyos/moteiv/tos/platform/msp430/adc/RefVolt.nc"
inline static void RefVoltM$RefVolt$isStable(RefVolt_t vref){
#line 127
  MSP430ADC12M$RefVolt$isStable(vref);
#line 127
}
#line 127
# 134 "/Users/jingyuancheng/tinyos/moteiv/tos/platform/msp430/adc/RefVoltM.nc"
static inline void RefVoltM$SwitchOnTimer$fired(void )
#line 134
{
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 135
    {
      if (RefVoltM$state == RefVoltM$REFERENCE_1_5V_PENDING) {
        RefVoltM$state = RefVoltM$REFERENCE_1_5V_STABLE;
        }
#line 138
      if (RefVoltM$state == RefVoltM$REFERENCE_2_5V_PENDING) {
        RefVoltM$state = RefVoltM$REFERENCE_2_5V_STABLE;
        }
    }
#line 141
    __nesc_atomic_end(__nesc_atomic); }
#line 141
  if (RefVoltM$state == RefVoltM$REFERENCE_1_5V_STABLE) {
    RefVoltM$RefVolt$isStable(REFERENCE_1_5V);
    }
#line 143
  if (RefVoltM$state == RefVoltM$REFERENCE_2_5V_STABLE) {
    RefVoltM$RefVolt$isStable(REFERENCE_2_5V);
    }
}

# 49 "/Users/jingyuancheng/tinyos/moteiv/tos/lib/sched/TaskBasic.nc"
inline static result_t RefVoltM$switchOffRetry$postTask(void ){
#line 49
  unsigned char __nesc_result;
#line 49

#line 49
  __nesc_result = SchedulerBasicP$TaskBasic$postTask(RefVoltM$switchOffRetry);
#line 49

#line 49
  return __nesc_result;
#line 49
}
#line 49
# 138 "/Users/jingyuancheng/tinyos/moteiv/tos/platform/msp430/adc/HPLADC12M.nc"
static inline void HPLADC12M$HPLADC12$setRefOff(void )
#line 138
{
#line 138
  HPLADC12M$ADC12CTL0 &= ~0x0020;
}

# 73 "/Users/jingyuancheng/tinyos/moteiv/tos/platform/msp430/adc/HPLADC12.nc"
inline static void RefVoltM$HPLADC12$setRefOff(void ){
#line 73
  HPLADC12M$HPLADC12$setRefOff();
#line 73
}
#line 73







inline static void RefVoltM$HPLADC12$disableConversion(void ){
#line 80
  HPLADC12M$HPLADC12$disableConversion();
#line 80
}
#line 80
# 112 "/Users/jingyuancheng/tinyos/moteiv/tos/platform/msp430/adc/HPLADC12M.nc"
static inline bool HPLADC12M$HPLADC12$isBusy(void )
#line 112
{
#line 112
  return HPLADC12M$ADC12CTL1 & 0x0001;
}

# 65 "/Users/jingyuancheng/tinyos/moteiv/tos/platform/msp430/adc/HPLADC12.nc"
inline static bool RefVoltM$HPLADC12$isBusy(void ){
#line 65
  unsigned char __nesc_result;
#line 65

#line 65
  __nesc_result = HPLADC12M$HPLADC12$isBusy();
#line 65

#line 65
  return __nesc_result;
#line 65
}
#line 65
# 172 "/Users/jingyuancheng/tinyos/moteiv/tos/platform/msp430/adc/RefVoltM.nc"
static __inline void RefVoltM$switchRefOff(void )
#line 172
{
  result_t result;

  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 175
    {
      if (RefVoltM$switchOff == FALSE) {
        result = FAIL;
        }
      else {
#line 178
        if (RefVoltM$HPLADC12$isBusy()) {
            result = FAIL;
          }
        else {
            RefVoltM$HPLADC12$disableConversion();
            RefVoltM$HPLADC12$setRefOff();
            RefVoltM$state = RefVoltM$REFERENCE_OFF;
            result = SUCCESS;
          }
        }
    }
#line 188
    __nesc_atomic_end(__nesc_atomic); }
#line 188
  if (RefVoltM$switchOff == TRUE && result == FAIL) {
    RefVoltM$switchOffRetry$postTask();
    }
}










static inline void RefVoltM$SwitchOffTimer$fired(void )
#line 202
{
  RefVoltM$switchRefOff();
}

# 107 "/Users/jingyuancheng/tinyos/moteiv/tos/lib/timer/TimerWrapC.nc"
static inline result_t TimerWrapC$TimerMilli$default$fired(uint8_t id)
{
  return SUCCESS;
}

# 43 "/Users/jingyuancheng/tinyos/moteiv/tos/lib/timer/TimerMilli.nc"
inline static result_t TimerWrapC$TimerMilli$fired(uint8_t arg_0x1025b6ab8){
#line 43
  unsigned char __nesc_result;
#line 43

#line 43
    __nesc_result = TimerWrapC$TimerMilli$default$fired(arg_0x1025b6ab8);
#line 43

#line 43
  return __nesc_result;
#line 43
}
#line 43
# 43 "/Users/jingyuancheng/tinyos/moteiv/tos/platform/tmote/hardware.h"
static inline void TOSH_SEL_UTXD1_IOFUNC()
#line 43
{
#line 43
  static volatile uint8_t r __asm ("0x001B");

#line 43
  r &= ~(1 << 6);
}

# 155 "/Users/jingyuancheng/tinyos/moteiv/tos/platform/msp430/HPLUSART1M.nc"
static inline void HPLUSART1M$USARTControl$disableUARTTx(void )
#line 155
{
  HPLUSART1M$ME2 &= ~(1 << 5);
  TOSH_SEL_UTXD1_IOFUNC();
}

# 95 "/Users/jingyuancheng/tinyos/moteiv/tinyos-1.x/tos/platform/msp430/HPLUSARTControl.nc"
inline static void FramerP$USARTControl$disableUARTTx(void ){
#line 95
  HPLUSART1M$USARTControl$disableUARTTx();
#line 95
}
#line 95
# 249 "/Users/jingyuancheng/tinyos/moteiv/tos/platform/tmote/FramerP.nc"
static inline result_t FramerP$Timer$fired(void )
#line 249
{
  FramerP$USARTControl$disableUARTTx();
  return SUCCESS;
}

# 102 "/Users/jingyuancheng/tinyos/moteiv/tos/lib/timer/TimerWrapC.nc"
static inline result_t TimerWrapC$Timer$default$fired(uint8_t id)
{
  return SUCCESS;
}

# 73 "/Users/jingyuancheng/tinyos/moteiv/tinyos-1.x/tos/interfaces/Timer.nc"
inline static result_t TimerWrapC$Timer$fired(uint8_t arg_0x1025bac10){
#line 73
  unsigned char __nesc_result;
#line 73

#line 73
  switch (arg_0x1025bac10) {
#line 73
    case 1U:
#line 73
      __nesc_result = FramerP$Timer$fired();
#line 73
      break;
#line 73
    default:
#line 73
      __nesc_result = TimerWrapC$Timer$default$fired(arg_0x1025bac10);
#line 73
      break;
#line 73
    }
#line 73

#line 73
  return __nesc_result;
#line 73
}
#line 73
# 92 "/Users/jingyuancheng/tinyos/moteiv/tos/lib/timer/TimerWrapC.nc"
static inline void TimerWrapC$Timer2$fired(uint8_t id)
{
  int16_t _id = id;

#line 95
  if (_id < TimerWrapC$TIMER_END) {
    TimerWrapC$Timer$fired(id);
    }
  else {
#line 98
    TimerWrapC$TimerMilli$fired(id - TimerWrapC$MILLI_BEGIN);
    }
}

# 68 "/Users/jingyuancheng/tinyos/moteiv/tos/lib/timer/Timer2.nc"
inline static void /*HalTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC$0$Timer$fired(uint8_t arg_0x101f29108){
#line 68
  TimerWrapC$Timer2$fired(arg_0x101f29108);
#line 68
  switch (arg_0x101f29108) {
#line 68
    case /*ADC8IO14C.TimerMilliC*/TimerMilliC$0$TIMER_ID:
#line 68
      ADC8IO14P$Timer$fired();
#line 68
      break;
#line 68
    case /*CC2420SyncAlwaysOnC.TimerMilliC*/TimerMilliC$1$TIMER_ID:
#line 68
      CC2420AlwaysOnM$SanityTimer$fired();
#line 68
      break;
#line 68
    case /*RefVoltC.Timer1*/TimerMilliC$2$TIMER_ID:
#line 68
      RefVoltM$SwitchOnTimer$fired();
#line 68
      break;
#line 68
    case /*RefVoltC.Timer2*/TimerMilliC$3$TIMER_ID:
#line 68
      RefVoltM$SwitchOffTimer$fired();
#line 68
      break;
#line 68
  }
#line 68
}
#line 68
# 116 "/Users/jingyuancheng/tinyos/moteiv/tos/platform/msp430/adc/HPLADC12M.nc"
static inline void HPLADC12M$HPLADC12$startConversion(void )
#line 116
{
#line 116
  HPLADC12M$ADC12CTL0 |= 0x0001 + 0x0002;
}

# 81 "/Users/jingyuancheng/tinyos/moteiv/tos/platform/msp430/adc/HPLADC12.nc"
inline static void MSP430ADC12M$HPLADC12$startConversion(void ){
#line 81
  HPLADC12M$HPLADC12$startConversion();
#line 81
}
#line 81
# 18 "/Users/jingyuancheng/tinyos/moteiv/tos/platform/msp430/timer/TimerExclusive.nc"
inline static result_t MSP430ADC12M$TimerExclusive$startTimer(uint8_t rh){
#line 18
  unsigned char __nesc_result;
#line 18

#line 18
  __nesc_result = MSP430TimerAExclusiveM$TimerExclusive$startTimer(rh);
#line 18

#line 18
  return __nesc_result;
#line 18
}
#line 18
# 179 "/Users/jingyuancheng/tinyos/moteiv/tos/platform/msp430/adc/MSP430ADC12M.nc"
static inline result_t MSP430ADC12M$startTimerA(void )
{
  return MSP430ADC12M$TimerExclusive$startTimer(MSP430ADC12M$rh);
}

# 45 "/Users/jingyuancheng/tinyos/moteiv/tos/platform/msp430/timer/MSP430TimerCapComM.nc"
static inline uint16_t /*MSP430TimerC.MSP430TimerA1*/MSP430TimerCapComM$1$CC2int(/*MSP430TimerC.MSP430TimerA1*/MSP430TimerCapComM$1$CC_t x)
#line 45
{
#line 45
  union /*MSP430TimerC.MSP430TimerA1*/MSP430TimerCapComM$1$__nesc_unnamed4399 {
#line 45
    /*MSP430TimerC.MSP430TimerA1*/MSP430TimerCapComM$1$CC_t f;
#line 45
    uint16_t t;
  } 
#line 45
  c = { .f = x };

#line 45
  return c.t;
}

#line 88
static inline void /*MSP430TimerC.MSP430TimerA1*/MSP430TimerCapComM$1$Control$setControl(/*MSP430TimerC.MSP430TimerA1*/MSP430TimerCapComM$1$CC_t x)
{
  * (volatile uint16_t *)356U = /*MSP430TimerC.MSP430TimerA1*/MSP430TimerCapComM$1$CC2int(x);
}

# 34 "/Users/jingyuancheng/tinyos/moteiv/tos/platform/msp430/timer/MSP430TimerControl.nc"
inline static void MSP430TimerAExclusiveM$ControlA1$setControl(MSP430CompareControl_t control){
#line 34
  /*MSP430TimerC.MSP430TimerA1*/MSP430TimerCapComM$1$Control$setControl(control);
#line 34
}
#line 34
# 40 "/Users/jingyuancheng/tinyos/moteiv/tinyos-1.x/tos/platform/msp430/MSP430GeneralIO.h"
static inline void TOSH_SET_PORT23_PIN()
#line 40
{
#line 40
  static volatile uint8_t r __asm ("0x0029");

#line 40
  r |= 1 << 3;
}

# 195 "/Users/jingyuancheng/tinyos/moteiv/tinyos-1.x/tos/platform/msp430/MSP430GeneralIOM.nc"
static inline void MSP430GeneralIOM$Port23$setHigh(void )
#line 195
{
#line 195
  TOSH_SET_PORT23_PIN();
}

# 27 "/Users/jingyuancheng/tinyos/moteiv/tinyos-1.x/tos/platform/msp430/MSP430GeneralIO.nc"
inline static void ADC8IO14P$Ctrl0$setHigh(void ){
#line 27
  MSP430GeneralIOM$Port23$setHigh();
#line 27
}
#line 27
# 40 "/Users/jingyuancheng/tinyos/moteiv/tinyos-1.x/tos/platform/msp430/MSP430GeneralIO.h"
static inline void TOSH_CLR_PORT23_PIN()
#line 40
{
#line 40
  static volatile uint8_t r __asm ("0x0029");

#line 40
  r &= ~(1 << 3);
}

# 196 "/Users/jingyuancheng/tinyos/moteiv/tinyos-1.x/tos/platform/msp430/MSP430GeneralIOM.nc"
static inline void MSP430GeneralIOM$Port23$setLow(void )
#line 196
{
#line 196
  TOSH_CLR_PORT23_PIN();
}

# 28 "/Users/jingyuancheng/tinyos/moteiv/tinyos-1.x/tos/platform/msp430/MSP430GeneralIO.nc"
inline static void ADC8IO14P$Ctrl0$setLow(void ){
#line 28
  MSP430GeneralIOM$Port23$setLow();
#line 28
}
#line 28
# 34 "/Users/jingyuancheng/tinyos/moteiv/tinyos-1.x/tos/platform/msp430/MSP430GeneralIO.h"
static inline void TOSH_SET_PORT16_PIN()
#line 34
{
#line 34
  static volatile uint8_t r __asm ("0x0021");

#line 34
  r |= 1 << 6;
}

# 145 "/Users/jingyuancheng/tinyos/moteiv/tinyos-1.x/tos/platform/msp430/MSP430GeneralIOM.nc"
static inline void MSP430GeneralIOM$Port16$setHigh(void )
#line 145
{
#line 145
  TOSH_SET_PORT16_PIN();
}

# 27 "/Users/jingyuancheng/tinyos/moteiv/tinyos-1.x/tos/platform/msp430/MSP430GeneralIO.nc"
inline static void ADC8IO14P$Ctrl1$setHigh(void ){
#line 27
  MSP430GeneralIOM$Port16$setHigh();
#line 27
}
#line 27
# 34 "/Users/jingyuancheng/tinyos/moteiv/tinyos-1.x/tos/platform/msp430/MSP430GeneralIO.h"
static inline void TOSH_CLR_PORT16_PIN()
#line 34
{
#line 34
  static volatile uint8_t r __asm ("0x0021");

#line 34
  r &= ~(1 << 6);
}

# 146 "/Users/jingyuancheng/tinyos/moteiv/tinyos-1.x/tos/platform/msp430/MSP430GeneralIOM.nc"
static inline void MSP430GeneralIOM$Port16$setLow(void )
#line 146
{
#line 146
  TOSH_CLR_PORT16_PIN();
}

# 28 "/Users/jingyuancheng/tinyos/moteiv/tinyos-1.x/tos/platform/msp430/MSP430GeneralIO.nc"
inline static void ADC8IO14P$Ctrl1$setLow(void ){
#line 28
  MSP430GeneralIOM$Port16$setLow();
#line 28
}
#line 28
# 38 "/Users/jingyuancheng/tinyos/moteiv/tinyos-1.x/tos/platform/msp430/MSP430GeneralIO.h"
static inline void TOSH_SET_PORT21_PIN()
#line 38
{
#line 38
  static volatile uint8_t r __asm ("0x0029");

#line 38
  r |= 1 << 1;
}

# 175 "/Users/jingyuancheng/tinyos/moteiv/tinyos-1.x/tos/platform/msp430/MSP430GeneralIOM.nc"
static inline void MSP430GeneralIOM$Port21$setHigh(void )
#line 175
{
#line 175
  TOSH_SET_PORT21_PIN();
}

# 27 "/Users/jingyuancheng/tinyos/moteiv/tinyos-1.x/tos/platform/msp430/MSP430GeneralIO.nc"
inline static void ADC8IO14P$Ctrl2$setHigh(void ){
#line 27
  MSP430GeneralIOM$Port21$setHigh();
#line 27
}
#line 27
# 38 "/Users/jingyuancheng/tinyos/moteiv/tinyos-1.x/tos/platform/msp430/MSP430GeneralIO.h"
static inline void TOSH_CLR_PORT21_PIN()
#line 38
{
#line 38
  static volatile uint8_t r __asm ("0x0029");

#line 38
  r &= ~(1 << 1);
}

# 176 "/Users/jingyuancheng/tinyos/moteiv/tinyos-1.x/tos/platform/msp430/MSP430GeneralIOM.nc"
static inline void MSP430GeneralIOM$Port21$setLow(void )
#line 176
{
#line 176
  TOSH_CLR_PORT21_PIN();
}

# 28 "/Users/jingyuancheng/tinyos/moteiv/tinyos-1.x/tos/platform/msp430/MSP430GeneralIO.nc"
inline static void ADC8IO14P$Ctrl2$setLow(void ){
#line 28
  MSP430GeneralIOM$Port21$setLow();
#line 28
}
#line 28
# 69 "/Users/jingyuancheng/tinyos/moteiv/tinyos-1.x/tos/platform/msp430/MSP430GeneralIO.h"
static inline void TOSH_SET_PORT55_PIN()
#line 69
{
#line 69
  static volatile uint8_t r __asm ("0x0031");

#line 69
  r |= 1 << 5;
}

# 455 "/Users/jingyuancheng/tinyos/moteiv/tinyos-1.x/tos/platform/msp430/MSP430GeneralIOM.nc"
static inline void MSP430GeneralIOM$Port55$setHigh(void )
#line 455
{
#line 455
  TOSH_SET_PORT55_PIN();
}

# 27 "/Users/jingyuancheng/tinyos/moteiv/tinyos-1.x/tos/platform/msp430/MSP430GeneralIO.nc"
inline static void ADC8IO14P$Ctrl3$setHigh(void ){
#line 27
  MSP430GeneralIOM$Port55$setHigh();
#line 27
}
#line 27
# 69 "/Users/jingyuancheng/tinyos/moteiv/tinyos-1.x/tos/platform/msp430/MSP430GeneralIO.h"
static inline void TOSH_CLR_PORT55_PIN()
#line 69
{
#line 69
  static volatile uint8_t r __asm ("0x0031");

#line 69
  r &= ~(1 << 5);
}

# 456 "/Users/jingyuancheng/tinyos/moteiv/tinyos-1.x/tos/platform/msp430/MSP430GeneralIOM.nc"
static inline void MSP430GeneralIOM$Port55$setLow(void )
#line 456
{
#line 456
  TOSH_CLR_PORT55_PIN();
}

# 28 "/Users/jingyuancheng/tinyos/moteiv/tinyos-1.x/tos/platform/msp430/MSP430GeneralIO.nc"
inline static void ADC8IO14P$Ctrl3$setLow(void ){
#line 28
  MSP430GeneralIOM$Port55$setLow();
#line 28
}
#line 28
# 68 "/Users/jingyuancheng/tinyos/moteiv/tinyos-1.x/tos/platform/msp430/MSP430GeneralIO.h"
static inline void TOSH_SET_PORT54_PIN()
#line 68
{
#line 68
  static volatile uint8_t r __asm ("0x0031");

#line 68
  r |= 1 << 4;
}

# 445 "/Users/jingyuancheng/tinyos/moteiv/tinyos-1.x/tos/platform/msp430/MSP430GeneralIOM.nc"
static inline void MSP430GeneralIOM$Port54$setHigh(void )
#line 445
{
#line 445
  TOSH_SET_PORT54_PIN();
}

# 27 "/Users/jingyuancheng/tinyos/moteiv/tinyos-1.x/tos/platform/msp430/MSP430GeneralIO.nc"
inline static void ADC8IO14P$Ctrl4$setHigh(void ){
#line 27
  MSP430GeneralIOM$Port54$setHigh();
#line 27
}
#line 27
# 68 "/Users/jingyuancheng/tinyos/moteiv/tinyos-1.x/tos/platform/msp430/MSP430GeneralIO.h"
static inline void TOSH_CLR_PORT54_PIN()
#line 68
{
#line 68
  static volatile uint8_t r __asm ("0x0031");

#line 68
  r &= ~(1 << 4);
}

# 446 "/Users/jingyuancheng/tinyos/moteiv/tinyos-1.x/tos/platform/msp430/MSP430GeneralIOM.nc"
static inline void MSP430GeneralIOM$Port54$setLow(void )
#line 446
{
#line 446
  TOSH_CLR_PORT54_PIN();
}

# 28 "/Users/jingyuancheng/tinyos/moteiv/tinyos-1.x/tos/platform/msp430/MSP430GeneralIO.nc"
inline static void ADC8IO14P$Ctrl4$setLow(void ){
#line 28
  MSP430GeneralIOM$Port54$setLow();
#line 28
}
#line 28
# 37 "/Users/jingyuancheng/tinyos/moteiv/tinyos-1.x/tos/platform/msp430/MSP430GeneralIO.h"
static inline void TOSH_SET_PORT20_PIN()
#line 37
{
#line 37
  static volatile uint8_t r __asm ("0x0029");

#line 37
  r |= 1 << 0;
}

# 165 "/Users/jingyuancheng/tinyos/moteiv/tinyos-1.x/tos/platform/msp430/MSP430GeneralIOM.nc"
static inline void MSP430GeneralIOM$Port20$setHigh(void )
#line 165
{
#line 165
  TOSH_SET_PORT20_PIN();
}

# 27 "/Users/jingyuancheng/tinyos/moteiv/tinyos-1.x/tos/platform/msp430/MSP430GeneralIO.nc"
inline static void ADC8IO14P$Ctrl5$setHigh(void ){
#line 27
  MSP430GeneralIOM$Port20$setHigh();
#line 27
}
#line 27
# 37 "/Users/jingyuancheng/tinyos/moteiv/tinyos-1.x/tos/platform/msp430/MSP430GeneralIO.h"
static inline void TOSH_CLR_PORT20_PIN()
#line 37
{
#line 37
  static volatile uint8_t r __asm ("0x0029");

#line 37
  r &= ~(1 << 0);
}

# 166 "/Users/jingyuancheng/tinyos/moteiv/tinyos-1.x/tos/platform/msp430/MSP430GeneralIOM.nc"
static inline void MSP430GeneralIOM$Port20$setLow(void )
#line 166
{
#line 166
  TOSH_CLR_PORT20_PIN();
}

# 28 "/Users/jingyuancheng/tinyos/moteiv/tinyos-1.x/tos/platform/msp430/MSP430GeneralIO.nc"
inline static void ADC8IO14P$Ctrl5$setLow(void ){
#line 28
  MSP430GeneralIOM$Port20$setLow();
#line 28
}
#line 28
# 33 "/Users/jingyuancheng/tinyos/moteiv/tinyos-1.x/tos/platform/msp430/MSP430GeneralIO.h"
static inline void TOSH_SET_PORT15_PIN()
#line 33
{
#line 33
  static volatile uint8_t r __asm ("0x0021");

#line 33
  r |= 1 << 5;
}

# 135 "/Users/jingyuancheng/tinyos/moteiv/tinyos-1.x/tos/platform/msp430/MSP430GeneralIOM.nc"
static inline void MSP430GeneralIOM$Port15$setHigh(void )
#line 135
{
#line 135
  TOSH_SET_PORT15_PIN();
}

# 27 "/Users/jingyuancheng/tinyos/moteiv/tinyos-1.x/tos/platform/msp430/MSP430GeneralIO.nc"
inline static void ADC8IO14P$Ctrl6$setHigh(void ){
#line 27
  MSP430GeneralIOM$Port15$setHigh();
#line 27
}
#line 27
# 33 "/Users/jingyuancheng/tinyos/moteiv/tinyos-1.x/tos/platform/msp430/MSP430GeneralIO.h"
static inline void TOSH_CLR_PORT15_PIN()
#line 33
{
#line 33
  static volatile uint8_t r __asm ("0x0021");

#line 33
  r &= ~(1 << 5);
}

# 136 "/Users/jingyuancheng/tinyos/moteiv/tinyos-1.x/tos/platform/msp430/MSP430GeneralIOM.nc"
static inline void MSP430GeneralIOM$Port15$setLow(void )
#line 136
{
#line 136
  TOSH_CLR_PORT15_PIN();
}

# 28 "/Users/jingyuancheng/tinyos/moteiv/tinyos-1.x/tos/platform/msp430/MSP430GeneralIO.nc"
inline static void ADC8IO14P$Ctrl6$setLow(void ){
#line 28
  MSP430GeneralIOM$Port15$setLow();
#line 28
}
#line 28
# 43 "/Users/jingyuancheng/tinyos/moteiv/tinyos-1.x/tos/platform/msp430/MSP430GeneralIO.h"
static inline void TOSH_SET_PORT26_PIN()
#line 43
{
#line 43
  static volatile uint8_t r __asm ("0x0029");

#line 43
  r |= 1 << 6;
}

# 225 "/Users/jingyuancheng/tinyos/moteiv/tinyos-1.x/tos/platform/msp430/MSP430GeneralIOM.nc"
static inline void MSP430GeneralIOM$Port26$setHigh(void )
#line 225
{
#line 225
  TOSH_SET_PORT26_PIN();
}

# 27 "/Users/jingyuancheng/tinyos/moteiv/tinyos-1.x/tos/platform/msp430/MSP430GeneralIO.nc"
inline static void ADC8IO14P$Ctrl7$setHigh(void ){
#line 27
  MSP430GeneralIOM$Port26$setHigh();
#line 27
}
#line 27
# 43 "/Users/jingyuancheng/tinyos/moteiv/tinyos-1.x/tos/platform/msp430/MSP430GeneralIO.h"
static inline void TOSH_CLR_PORT26_PIN()
#line 43
{
#line 43
  static volatile uint8_t r __asm ("0x0029");

#line 43
  r &= ~(1 << 6);
}

# 226 "/Users/jingyuancheng/tinyos/moteiv/tinyos-1.x/tos/platform/msp430/MSP430GeneralIOM.nc"
static inline void MSP430GeneralIOM$Port26$setLow(void )
#line 226
{
#line 226
  TOSH_CLR_PORT26_PIN();
}

# 28 "/Users/jingyuancheng/tinyos/moteiv/tinyos-1.x/tos/platform/msp430/MSP430GeneralIO.nc"
inline static void ADC8IO14P$Ctrl7$setLow(void ){
#line 28
  MSP430GeneralIOM$Port26$setLow();
#line 28
}
#line 28
# 206 "/Users/jingyuancheng/tinyos/moteiv/tos/platform/msp430/adc/RefVoltM.nc"
static inline RefVolt_t RefVoltM$RefVolt$getState(void )
#line 206
{
  if (RefVoltM$state == RefVoltM$REFERENCE_2_5V_STABLE) {
    return REFERENCE_2_5V;
    }
#line 209
  if (RefVoltM$state == RefVoltM$REFERENCE_1_5V_STABLE) {
    return REFERENCE_1_5V;
    }
#line 211
  return REFERENCE_UNSTABLE;
}

# 118 "/Users/jingyuancheng/tinyos/moteiv/tos/platform/msp430/adc/RefVolt.nc"
inline static RefVolt_t MSP430ADC12M$RefVolt$getState(void ){
#line 118
  enum __nesc_unnamed4300 __nesc_result;
#line 118

#line 118
  __nesc_result = RefVoltM$RefVolt$getState();
#line 118

#line 118
  return __nesc_result;
#line 118
}
#line 118
#line 92
inline static result_t MSP430ADC12M$RefVolt$get(RefVolt_t vref){
#line 92
  unsigned char __nesc_result;
#line 92

#line 92
  __nesc_result = RefVoltM$RefVolt$get(vref);
#line 92

#line 92
  return __nesc_result;
#line 92
}
#line 92
# 131 "/Users/jingyuancheng/tinyos/moteiv/tos/platform/msp430/adc/MSP430ADC12M.nc"
static inline msp430ADCresult_t MSP430ADC12M$getRefVolt(uint8_t num)
{
  msp430ADCresult_t adcResult = MSP430ADC12_SUCCESS;
  result_t vrefResult;
  adc12memctl_t memctl = MSP430ADC12M$adc12settings[num].memctl;

  if (memctl.sref == REFERENCE_VREFplus_AVss || 
  memctl.sref == REFERENCE_VREFplus_VREFnegterm) 
    {
      if (MSP430ADC12M$adc12settings[num].gotRefVolt == 0) {
          if (MSP430ADC12M$adc12settings[num].refVolt2_5) {
            vrefResult = MSP430ADC12M$RefVolt$get(REFERENCE_2_5V);
            }
          else {
#line 144
            vrefResult = MSP430ADC12M$RefVolt$get(REFERENCE_1_5V);
            }
        }
      else {
#line 146
        vrefResult = SUCCESS;
        }
#line 147
      if (vrefResult != SUCCESS) 
        {
          adcResult = MSP430ADC12_FAIL;
        }
      else 
#line 150
        {
          MSP430ADC12M$adc12settings[num].gotRefVolt = 1;
          if (MSP430ADC12M$RefVolt$getState() == REFERENCE_UNSTABLE) {
            adcResult = MSP430ADC12_DELAYED;
            }
        }
    }
#line 156
  return adcResult;
}

# 49 "/Users/jingyuancheng/tinyos/moteiv/tos/lib/sched/TaskBasic.nc"
inline static result_t RefVoltM$switchOnDelay$postTask(void ){
#line 49
  unsigned char __nesc_result;
#line 49

#line 49
  __nesc_result = SchedulerBasicP$TaskBasic$postTask(RefVoltM$switchOnDelay);
#line 49

#line 49
  return __nesc_result;
#line 49
}
#line 49
# 141 "/Users/jingyuancheng/tinyos/moteiv/tos/platform/msp430/adc/HPLADC12M.nc"
static inline void HPLADC12M$HPLADC12$setRef2_5V(void )
#line 141
{
#line 141
  HPLADC12M$ADC12CTL0 |= 0x0040;
}

# 76 "/Users/jingyuancheng/tinyos/moteiv/tos/platform/msp430/adc/HPLADC12.nc"
inline static void RefVoltM$HPLADC12$setRef2_5V(void ){
#line 76
  HPLADC12M$HPLADC12$setRef2_5V();
#line 76
}
#line 76
# 140 "/Users/jingyuancheng/tinyos/moteiv/tos/platform/msp430/adc/HPLADC12M.nc"
static inline void HPLADC12M$HPLADC12$setRef1_5V(void )
#line 140
{
#line 140
  HPLADC12M$ADC12CTL0 &= ~0x0040;
}

# 75 "/Users/jingyuancheng/tinyos/moteiv/tos/platform/msp430/adc/HPLADC12.nc"
inline static void RefVoltM$HPLADC12$setRef1_5V(void ){
#line 75
  HPLADC12M$HPLADC12$setRef1_5V();
#line 75
}
#line 75
# 137 "/Users/jingyuancheng/tinyos/moteiv/tos/platform/msp430/adc/HPLADC12M.nc"
static inline void HPLADC12M$HPLADC12$setRefOn(void )
#line 137
{
#line 137
  HPLADC12M$ADC12CTL0 |= 0x0020;
}

# 72 "/Users/jingyuancheng/tinyos/moteiv/tos/platform/msp430/adc/HPLADC12.nc"
inline static void RefVoltM$HPLADC12$setRefOn(void ){
#line 72
  HPLADC12M$HPLADC12$setRefOn();
#line 72
}
#line 72
# 108 "/Users/jingyuancheng/tinyos/moteiv/tos/platform/msp430/adc/RefVoltM.nc"
static __inline void RefVoltM$switchRefOn(uint8_t vref)
#line 108
{
  RefVoltM$HPLADC12$disableConversion();
  RefVoltM$HPLADC12$setRefOn();
  if (vref == REFERENCE_1_5V) {
      RefVoltM$HPLADC12$setRef1_5V();
      /* atomic removed: atomic calls only */
#line 113
      RefVoltM$state = RefVoltM$REFERENCE_1_5V_PENDING;
    }
  else {
      RefVoltM$HPLADC12$setRef2_5V();
      /* atomic removed: atomic calls only */
#line 117
      RefVoltM$state = RefVoltM$REFERENCE_2_5V_PENDING;
    }
  RefVoltM$switchOnDelay$postTask();
}

static __inline void RefVoltM$switchToRefPending(uint8_t vref)
#line 122
{
  RefVoltM$switchRefOn(vref);
}

static __inline void RefVoltM$switchToRefStable(uint8_t vref)
#line 126
{
  RefVoltM$switchRefOn(vref);
}

# 58 "/Users/jingyuancheng/tinyos/moteiv/tos/platform/msp430/adc/HPLADC12M.nc"
static inline void HPLADC12M$HPLADC12$setControl0_IgnoreRef(adc12ctl0_t control0)
#line 58
{
  adc12ctl0_t oldControl0 = * (adc12ctl0_t *)&HPLADC12M$ADC12CTL0;

#line 60
  control0.refon = oldControl0.refon;
  control0.r2_5v = oldControl0.r2_5v;
  HPLADC12M$ADC12CTL0 = * (uint16_t *)&control0;
}

# 48 "/Users/jingyuancheng/tinyos/moteiv/tos/platform/msp430/adc/HPLADC12.nc"
inline static void MSP430ADC12M$HPLADC12$setControl0_IgnoreRef(adc12ctl0_t control0){
#line 48
  HPLADC12M$HPLADC12$setControl0_IgnoreRef(control0);
#line 48
}
#line 48
# 144 "/Users/jingyuancheng/tinyos/moteiv/tos/platform/msp430/adc/HPLADC12M.nc"
static inline void HPLADC12M$HPLADC12$setSHT(uint8_t sht)
#line 144
{
  uint16_t ctl0 = HPLADC12M$ADC12CTL0;
  uint16_t shttemp = sht & 0x0F;

#line 147
  ctl0 &= 0x00FF;
  ctl0 |= shttemp << 8;
  ctl0 |= shttemp << 12;
  HPLADC12M$ADC12CTL0 = ctl0;
}

# 69 "/Users/jingyuancheng/tinyos/moteiv/tos/platform/msp430/adc/HPLADC12.nc"
inline static void MSP430ADC12M$HPLADC12$setSHT(uint8_t sht){
#line 69
  HPLADC12M$HPLADC12$setSHT(sht);
#line 69
}
#line 69
# 54 "/Users/jingyuancheng/tinyos/moteiv/tos/platform/msp430/adc/HPLADC12M.nc"
static inline void HPLADC12M$HPLADC12$setControl1(adc12ctl1_t control1)
#line 54
{
  HPLADC12M$ADC12CTL1 = * (uint16_t *)&control1;
}

# 43 "/Users/jingyuancheng/tinyos/moteiv/tos/platform/msp430/adc/HPLADC12.nc"
inline static void MSP430ADC12M$HPLADC12$setControl1(adc12ctl1_t control1){
#line 43
  HPLADC12M$HPLADC12$setControl1(control1);
#line 43
}
#line 43
# 31 "/Users/jingyuancheng/tinyos/moteiv/tos/platform/msp430/resource/MSP430ResourceConfigTimerAP.nc"
static inline void MSP430ResourceConfigTimerAP$Arbiter$requested(void )
#line 31
{
}

# 54 "/Users/jingyuancheng/tinyos/moteiv/tos/lib/resource/Arbiter.nc"
inline static void /*MSP430ArbiterTimerAC.ArbiterC.ArbiterP*/FcfsArbiterP$0$Arbiter$requested(void ){
#line 54
  MSP430ResourceConfigTimerAP$Arbiter$requested();
#line 54
}
#line 54
# 204 "/Users/jingyuancheng/tinyos/moteiv/tos/lib/resource/FcfsArbiterP.nc"
static inline uint8_t /*MSP430ArbiterTimerAC.ArbiterC.ArbiterP*/FcfsArbiterP$0$ResourceCmdAsync$immediateRequest(uint8_t id, uint8_t rh)
#line 204
{
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 205
    {
      if (/*MSP430ArbiterTimerAC.ArbiterC.ArbiterP*/FcfsArbiterP$0$m_granted == RESOURCE_NONE) {
          /*MSP430ArbiterTimerAC.ArbiterC.ArbiterP*/FcfsArbiterP$0$m_granted = rh = id;
          /*MSP430ArbiterTimerAC.ArbiterC.ArbiterP*/FcfsArbiterP$0$ResourceConfigure$configure(id);
        }
      else {
#line 210
        if (rh != /*MSP430ArbiterTimerAC.ArbiterC.ArbiterP*/FcfsArbiterP$0$m_granted) {
            rh = RESOURCE_NONE;
          }
        }
    }
#line 214
    __nesc_atomic_end(__nesc_atomic); }
#line 214
  if (rh == RESOURCE_NONE) {
    /*MSP430ArbiterTimerAC.ArbiterC.ArbiterP*/FcfsArbiterP$0$Arbiter$requested();
    }
#line 216
  return rh;
}

# 73 "/Users/jingyuancheng/tinyos/moteiv/tos/lib/resource/ResourceCmdAsync.nc"
inline static uint8_t MSP430ADC12M$Resource$immediateRequest(uint8_t rh){
#line 73
  unsigned char __nesc_result;
#line 73

#line 73
  __nesc_result = /*MSP430ArbiterTimerAC.ArbiterC.ArbiterP*/FcfsArbiterP$0$ResourceCmdAsync$immediateRequest(/*MSP430ADC12C.ResourceC*/MSP430ResourceTimerAC$1$ID, rh);
#line 73

#line 73
  return __nesc_result;
#line 73
}
#line 73
# 35 "/Users/jingyuancheng/tinyos/moteiv/tos/platform/msp430/timer/MSP430Timer.nc"
inline static void MSP430ResourceConfigTimerAP$TimerA$setMode(int mode){
#line 35
  /*MSP430TimerC.MSP430TimerA*/MSP430TimerM$0$Timer$setMode(mode);
#line 35
}
#line 35
# 143 "/Users/jingyuancheng/tinyos/moteiv/tos/platform/msp430/timer/MSP430TimerCapComM.nc"
static inline void /*MSP430TimerC.MSP430TimerA1*/MSP430TimerCapComM$1$Compare$setEvent(uint16_t x)
{
  * (volatile uint16_t *)372U = x;
}

# 30 "/Users/jingyuancheng/tinyos/moteiv/tos/platform/msp430/timer/MSP430Compare.nc"
inline static void MSP430TimerAExclusiveM$CompareA1$setEvent(uint16_t time){
#line 30
  /*MSP430TimerC.MSP430TimerA1*/MSP430TimerCapComM$1$Compare$setEvent(time);
#line 30
}
#line 30
# 143 "/Users/jingyuancheng/tinyos/moteiv/tos/platform/msp430/timer/MSP430TimerCapComM.nc"
static inline void /*MSP430TimerC.MSP430TimerA0*/MSP430TimerCapComM$0$Compare$setEvent(uint16_t x)
{
  * (volatile uint16_t *)370U = x;
}

# 30 "/Users/jingyuancheng/tinyos/moteiv/tos/platform/msp430/timer/MSP430Compare.nc"
inline static void MSP430TimerAExclusiveM$CompareA0$setEvent(uint16_t time){
#line 30
  /*MSP430TimerC.MSP430TimerA0*/MSP430TimerCapComM$0$Compare$setEvent(time);
#line 30
}
#line 30
# 45 "/Users/jingyuancheng/tinyos/moteiv/tos/platform/msp430/timer/MSP430TimerCapComM.nc"
static inline uint16_t /*MSP430TimerC.MSP430TimerA0*/MSP430TimerCapComM$0$CC2int(/*MSP430TimerC.MSP430TimerA0*/MSP430TimerCapComM$0$CC_t x)
#line 45
{
#line 45
  union /*MSP430TimerC.MSP430TimerA0*/MSP430TimerCapComM$0$__nesc_unnamed4400 {
#line 45
    /*MSP430TimerC.MSP430TimerA0*/MSP430TimerCapComM$0$CC_t f;
#line 45
    uint16_t t;
  } 
#line 45
  c = { .f = x };

#line 45
  return c.t;
}

#line 88
static inline void /*MSP430TimerC.MSP430TimerA0*/MSP430TimerCapComM$0$Control$setControl(/*MSP430TimerC.MSP430TimerA0*/MSP430TimerCapComM$0$CC_t x)
{
  * (volatile uint16_t *)354U = /*MSP430TimerC.MSP430TimerA0*/MSP430TimerCapComM$0$CC2int(x);
}

# 34 "/Users/jingyuancheng/tinyos/moteiv/tos/platform/msp430/timer/MSP430TimerControl.nc"
inline static void MSP430TimerAExclusiveM$ControlA0$setControl(MSP430CompareControl_t control){
#line 34
  /*MSP430TimerC.MSP430TimerA0*/MSP430TimerCapComM$0$Control$setControl(control);
#line 34
}
#line 34
# 41 "/Users/jingyuancheng/tinyos/moteiv/tos/platform/msp430/timer/MSP430Timer.nc"
inline static void MSP430TimerAExclusiveM$TimerA$setInputDivider(uint16_t inputDivider){
#line 41
  /*MSP430TimerC.MSP430TimerA*/MSP430TimerM$0$Timer$setInputDivider(inputDivider);
#line 41
}
#line 41
#line 40
inline static void MSP430TimerAExclusiveM$TimerA$setClockSource(uint16_t clockSource){
#line 40
  /*MSP430TimerC.MSP430TimerA*/MSP430TimerM$0$Timer$setClockSource(clockSource);
#line 40
}
#line 40
# 89 "/Users/jingyuancheng/tinyos/moteiv/tos/platform/msp430/timer/MSP430TimerM.nc"
static inline void /*MSP430TimerC.MSP430TimerA*/MSP430TimerM$0$Timer$clear(void )
{
  * (volatile uint16_t *)352U |= 4U;
}

# 37 "/Users/jingyuancheng/tinyos/moteiv/tos/platform/msp430/timer/MSP430Timer.nc"
inline static void MSP430TimerAExclusiveM$TimerA$clear(void ){
#line 37
  /*MSP430TimerC.MSP430TimerA*/MSP430TimerM$0$Timer$clear();
#line 37
}
#line 37
#line 35
inline static void MSP430TimerAExclusiveM$TimerA$setMode(int mode){
#line 35
  /*MSP430TimerC.MSP430TimerA*/MSP430TimerM$0$Timer$setMode(mode);
#line 35
}
#line 35




inline static void MSP430TimerAExclusiveM$TimerA$disableEvents(void ){
#line 39
  /*MSP430TimerC.MSP430TimerA*/MSP430TimerM$0$Timer$disableEvents();
#line 39
}
#line 39
# 28 "/Users/jingyuancheng/tinyos/moteiv/tos/lib/resource/ResourceValidate.nc"
inline static bool MSP430TimerAExclusiveM$ResourceValidate$validateUser(uint8_t rh){
#line 28
  unsigned char __nesc_result;
#line 28

#line 28
  __nesc_result = /*MSP430ArbiterTimerAC.ArbiterC.ArbiterP*/FcfsArbiterP$0$ResourceValidate$validateUser(rh);
#line 28

#line 28
  return __nesc_result;
#line 28
}
#line 28
# 52 "/Users/jingyuancheng/tinyos/moteiv/tos/platform/msp430/timer/MSP430TimerAExclusiveM.nc"
static inline result_t MSP430TimerAExclusiveM$TimerExclusive$prepareTimer(uint8_t rh, uint16_t interval, uint16_t csSAMPCON, uint16_t cdSAMPCON)
{

  MSP430CompareControl_t ccResetSHI = { 
  .ccifg = 0, .cov = 0, .out = 0, .cci = 0, .ccie = 0, 
  .outmod = 0, .cap = 0, .clld = 0, .scs = 0, .ccis = 0, .cm = 0 };

  if (!MSP430TimerAExclusiveM$ResourceValidate$validateUser(rh)) {
    return FAIL;
    }
  MSP430TimerAExclusiveM$TimerA$disableEvents();
  MSP430TimerAExclusiveM$TimerA$setMode(MSP430TIMER_STOP_MODE);
  MSP430TimerAExclusiveM$TimerA$clear();
  MSP430TimerAExclusiveM$TimerA$setClockSource(csSAMPCON);
  MSP430TimerAExclusiveM$TimerA$setInputDivider(cdSAMPCON);
  MSP430TimerAExclusiveM$ControlA0$setControl(ccResetSHI);
  MSP430TimerAExclusiveM$CompareA0$setEvent(interval - 1);
  MSP430TimerAExclusiveM$CompareA1$setEvent((interval - 1) / 2);
  return SUCCESS;
}

# 14 "/Users/jingyuancheng/tinyos/moteiv/tos/platform/msp430/timer/TimerExclusive.nc"
inline static result_t MSP430ADC12M$TimerExclusive$prepareTimer(uint8_t rh, uint16_t interval, uint16_t csSAMPCON, uint16_t cdSAMPCON){
#line 14
  unsigned char __nesc_result;
#line 14

#line 14
  __nesc_result = MSP430TimerAExclusiveM$TimerExclusive$prepareTimer(rh, interval, csSAMPCON, cdSAMPCON);
#line 14

#line 14
  return __nesc_result;
#line 14
}
#line 14
# 79 "/Users/jingyuancheng/tinyos/moteiv/tos/lib/timer/AlarmToTimerC.nc"
static inline void /*HalTimerMilliC.AlarmToTimerC*/AlarmToTimerC$0$Timer$startOneShotAt(uint32_t t0, uint32_t dt)
#line 79
{
  /*HalTimerMilliC.AlarmToTimerC*/AlarmToTimerC$0$start(t0, dt, TRUE);
}

# 111 "/Users/jingyuancheng/tinyos/moteiv/tos/lib/timer/Timer2.nc"
inline static void /*HalTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC$0$TimerFrom$startOneShotAt(uint32_t t0, uint32_t dt){
#line 111
  /*HalTimerMilliC.AlarmToTimerC*/AlarmToTimerC$0$Timer$startOneShotAt(t0, dt);
#line 111
}
#line 111
# 50 "/Users/jingyuancheng/tinyos/moteiv/tos/lib/sched/TaskBasic.nc"
inline static result_t /*MSP430ArbiterTimerAC.ArbiterC.ArbiterP*/FcfsArbiterP$0$GrantTask$postUrgentTask(void ){
#line 50
  unsigned char __nesc_result;
#line 50

#line 50
  __nesc_result = SchedulerBasicP$TaskBasic$postUrgentTask(0U);
#line 50

#line 50
  return __nesc_result;
#line 50
}
#line 50
# 27 "/Users/jingyuancheng/tinyos/moteiv/tos/platform/msp430/resource/MSP430ResourceConfigTimerAP.nc"
static inline void MSP430ResourceConfigTimerAP$Arbiter$idle(void )
#line 27
{
  MSP430ResourceConfigTimerAP$idle();
}

# 60 "/Users/jingyuancheng/tinyos/moteiv/tos/lib/resource/Arbiter.nc"
inline static void /*MSP430ArbiterTimerAC.ArbiterC.ArbiterP*/FcfsArbiterP$0$Arbiter$idle(void ){
#line 60
  MSP430ResourceConfigTimerAP$Arbiter$idle();
#line 60
}
#line 60
# 127 "/Users/jingyuancheng/tinyos/moteiv/tos/lib/sched/MainP.nc"
static inline void MainP$Boot$default$booted(void )
#line 127
{
}

# 40 "/Users/jingyuancheng/tinyos/moteiv/tos/lib/sched/Boot.nc"
inline static void MainP$Boot$booted(void ){
#line 40
  MainP$Boot$default$booted();
#line 40
}
#line 40
# 77 "/Users/jingyuancheng/tinyos/moteiv/tos/platform/msp430/adc/MSP430ADC12M.nc"
static inline result_t MSP430ADC12M$StdControl$start(void )
{
  MSP430ADC12M$rh = RESOURCE_NONE;
  return SUCCESS;
}

# 52 "/Users/jingyuancheng/tinyos/moteiv/tos/lib/timer/Timer2.nc"
inline static void ADC8IO14P$Timer$startPeriodic(uint32_t dt){
#line 52
  /*HalTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC$0$Timer$startPeriodic(/*ADC8IO14C.TimerMilliC*/TimerMilliC$0$TIMER_ID, dt);
#line 52
}
#line 52
# 70 "/Users/jingyuancheng/tinyos/moteiv/tinyos-1.x/tos/platform/msp430/MSP430GeneralIO.h"
static inline void TOSH_MAKE_PORT56_OUTPUT()
#line 70
{
#line 70
  static volatile uint8_t r __asm ("0x0032");

#line 70
  r |= 1 << 6;
}

# 471 "/Users/jingyuancheng/tinyos/moteiv/tinyos-1.x/tos/platform/msp430/MSP430GeneralIOM.nc"
static inline void MSP430GeneralIOM$Port56$makeOutput(void )
#line 471
{
#line 471
  TOSH_MAKE_PORT56_OUTPUT();
}

# 33 "/Users/jingyuancheng/tinyos/moteiv/tinyos-1.x/tos/platform/msp430/MSP430GeneralIO.nc"
inline static void ADC8IO14P$LedUsr$makeOutput(void ){
#line 33
  MSP430GeneralIOM$Port56$makeOutput();
#line 33
}
#line 33
# 43 "/Users/jingyuancheng/tinyos/moteiv/tinyos-1.x/tos/platform/msp430/MSP430GeneralIO.h"
static inline void TOSH_MAKE_PORT26_OUTPUT()
#line 43
{
#line 43
  static volatile uint8_t r __asm ("0x002A");

#line 43
  r |= 1 << 6;
}

# 231 "/Users/jingyuancheng/tinyos/moteiv/tinyos-1.x/tos/platform/msp430/MSP430GeneralIOM.nc"
static inline void MSP430GeneralIOM$Port26$makeOutput(void )
#line 231
{
#line 231
  TOSH_MAKE_PORT26_OUTPUT();
}

# 33 "/Users/jingyuancheng/tinyos/moteiv/tinyos-1.x/tos/platform/msp430/MSP430GeneralIO.nc"
inline static void ADC8IO14P$Ctrl7$makeOutput(void ){
#line 33
  MSP430GeneralIOM$Port26$makeOutput();
#line 33
}
#line 33
# 33 "/Users/jingyuancheng/tinyos/moteiv/tinyos-1.x/tos/platform/msp430/MSP430GeneralIO.h"
static inline void TOSH_MAKE_PORT15_OUTPUT()
#line 33
{
#line 33
  static volatile uint8_t r __asm ("0x0022");

#line 33
  r |= 1 << 5;
}

# 141 "/Users/jingyuancheng/tinyos/moteiv/tinyos-1.x/tos/platform/msp430/MSP430GeneralIOM.nc"
static inline void MSP430GeneralIOM$Port15$makeOutput(void )
#line 141
{
#line 141
  TOSH_MAKE_PORT15_OUTPUT();
}

# 33 "/Users/jingyuancheng/tinyos/moteiv/tinyos-1.x/tos/platform/msp430/MSP430GeneralIO.nc"
inline static void ADC8IO14P$Ctrl6$makeOutput(void ){
#line 33
  MSP430GeneralIOM$Port15$makeOutput();
#line 33
}
#line 33
# 37 "/Users/jingyuancheng/tinyos/moteiv/tinyos-1.x/tos/platform/msp430/MSP430GeneralIO.h"
static inline void TOSH_MAKE_PORT20_OUTPUT()
#line 37
{
#line 37
  static volatile uint8_t r __asm ("0x002A");

#line 37
  r |= 1 << 0;
}

# 171 "/Users/jingyuancheng/tinyos/moteiv/tinyos-1.x/tos/platform/msp430/MSP430GeneralIOM.nc"
static inline void MSP430GeneralIOM$Port20$makeOutput(void )
#line 171
{
#line 171
  TOSH_MAKE_PORT20_OUTPUT();
}

# 33 "/Users/jingyuancheng/tinyos/moteiv/tinyos-1.x/tos/platform/msp430/MSP430GeneralIO.nc"
inline static void ADC8IO14P$Ctrl5$makeOutput(void ){
#line 33
  MSP430GeneralIOM$Port20$makeOutput();
#line 33
}
#line 33
# 68 "/Users/jingyuancheng/tinyos/moteiv/tinyos-1.x/tos/platform/msp430/MSP430GeneralIO.h"
static inline void TOSH_MAKE_PORT54_OUTPUT()
#line 68
{
#line 68
  static volatile uint8_t r __asm ("0x0032");

#line 68
  r |= 1 << 4;
}

# 451 "/Users/jingyuancheng/tinyos/moteiv/tinyos-1.x/tos/platform/msp430/MSP430GeneralIOM.nc"
static inline void MSP430GeneralIOM$Port54$makeOutput(void )
#line 451
{
#line 451
  TOSH_MAKE_PORT54_OUTPUT();
}

# 33 "/Users/jingyuancheng/tinyos/moteiv/tinyos-1.x/tos/platform/msp430/MSP430GeneralIO.nc"
inline static void ADC8IO14P$Ctrl4$makeOutput(void ){
#line 33
  MSP430GeneralIOM$Port54$makeOutput();
#line 33
}
#line 33
# 69 "/Users/jingyuancheng/tinyos/moteiv/tinyos-1.x/tos/platform/msp430/MSP430GeneralIO.h"
static inline void TOSH_MAKE_PORT55_OUTPUT()
#line 69
{
#line 69
  static volatile uint8_t r __asm ("0x0032");

#line 69
  r |= 1 << 5;
}

# 461 "/Users/jingyuancheng/tinyos/moteiv/tinyos-1.x/tos/platform/msp430/MSP430GeneralIOM.nc"
static inline void MSP430GeneralIOM$Port55$makeOutput(void )
#line 461
{
#line 461
  TOSH_MAKE_PORT55_OUTPUT();
}

# 33 "/Users/jingyuancheng/tinyos/moteiv/tinyos-1.x/tos/platform/msp430/MSP430GeneralIO.nc"
inline static void ADC8IO14P$Ctrl3$makeOutput(void ){
#line 33
  MSP430GeneralIOM$Port55$makeOutput();
#line 33
}
#line 33
# 38 "/Users/jingyuancheng/tinyos/moteiv/tinyos-1.x/tos/platform/msp430/MSP430GeneralIO.h"
static inline void TOSH_MAKE_PORT21_OUTPUT()
#line 38
{
#line 38
  static volatile uint8_t r __asm ("0x002A");

#line 38
  r |= 1 << 1;
}

# 181 "/Users/jingyuancheng/tinyos/moteiv/tinyos-1.x/tos/platform/msp430/MSP430GeneralIOM.nc"
static inline void MSP430GeneralIOM$Port21$makeOutput(void )
#line 181
{
#line 181
  TOSH_MAKE_PORT21_OUTPUT();
}

# 33 "/Users/jingyuancheng/tinyos/moteiv/tinyos-1.x/tos/platform/msp430/MSP430GeneralIO.nc"
inline static void ADC8IO14P$Ctrl2$makeOutput(void ){
#line 33
  MSP430GeneralIOM$Port21$makeOutput();
#line 33
}
#line 33
# 34 "/Users/jingyuancheng/tinyos/moteiv/tinyos-1.x/tos/platform/msp430/MSP430GeneralIO.h"
static inline void TOSH_MAKE_PORT16_OUTPUT()
#line 34
{
#line 34
  static volatile uint8_t r __asm ("0x0022");

#line 34
  r |= 1 << 6;
}

# 151 "/Users/jingyuancheng/tinyos/moteiv/tinyos-1.x/tos/platform/msp430/MSP430GeneralIOM.nc"
static inline void MSP430GeneralIOM$Port16$makeOutput(void )
#line 151
{
#line 151
  TOSH_MAKE_PORT16_OUTPUT();
}

# 33 "/Users/jingyuancheng/tinyos/moteiv/tinyos-1.x/tos/platform/msp430/MSP430GeneralIO.nc"
inline static void ADC8IO14P$Ctrl1$makeOutput(void ){
#line 33
  MSP430GeneralIOM$Port16$makeOutput();
#line 33
}
#line 33
# 40 "/Users/jingyuancheng/tinyos/moteiv/tinyos-1.x/tos/platform/msp430/MSP430GeneralIO.h"
static inline void TOSH_MAKE_PORT23_OUTPUT()
#line 40
{
#line 40
  static volatile uint8_t r __asm ("0x002A");

#line 40
  r |= 1 << 3;
}

# 201 "/Users/jingyuancheng/tinyos/moteiv/tinyos-1.x/tos/platform/msp430/MSP430GeneralIOM.nc"
static inline void MSP430GeneralIOM$Port23$makeOutput(void )
#line 201
{
#line 201
  TOSH_MAKE_PORT23_OUTPUT();
}

# 33 "/Users/jingyuancheng/tinyos/moteiv/tinyos-1.x/tos/platform/msp430/MSP430GeneralIO.nc"
inline static void ADC8IO14P$Ctrl0$makeOutput(void ){
#line 33
  MSP430GeneralIOM$Port23$makeOutput();
#line 33
}
#line 33
# 222 "/Users/jingyuancheng/tinyos/moteiv/tos/platform/msp430/adc/MSP430ADC12.h"
static __inline MSP430ADC12Settings_t int2adcSettings(uint32_t i)
#line 222
{
  MSP430ADC12Settings_ut u;

#line 224
  u.i = i;
  return u.s;
}

# 52 "/Users/jingyuancheng/tinyos/moteiv/tos/platform/msp430/adc/MSP430ADC12Single.nc"
inline static result_t ADC8IO14P$ADC7$bind(MSP430ADC12Settings_t settings){
#line 52
  unsigned char __nesc_result;
#line 52

#line 52
  __nesc_result = MSP430ADC12M$ADCSingle$bind(7U, settings);
#line 52

#line 52
  return __nesc_result;
#line 52
}
#line 52
inline static result_t ADC8IO14P$ADC6$bind(MSP430ADC12Settings_t settings){
#line 52
  unsigned char __nesc_result;
#line 52

#line 52
  __nesc_result = MSP430ADC12M$ADCSingle$bind(6U, settings);
#line 52

#line 52
  return __nesc_result;
#line 52
}
#line 52
inline static result_t ADC8IO14P$ADC5$bind(MSP430ADC12Settings_t settings){
#line 52
  unsigned char __nesc_result;
#line 52

#line 52
  __nesc_result = MSP430ADC12M$ADCSingle$bind(5U, settings);
#line 52

#line 52
  return __nesc_result;
#line 52
}
#line 52
inline static result_t ADC8IO14P$ADC4$bind(MSP430ADC12Settings_t settings){
#line 52
  unsigned char __nesc_result;
#line 52

#line 52
  __nesc_result = MSP430ADC12M$ADCSingle$bind(4U, settings);
#line 52

#line 52
  return __nesc_result;
#line 52
}
#line 52
inline static result_t ADC8IO14P$ADC3$bind(MSP430ADC12Settings_t settings){
#line 52
  unsigned char __nesc_result;
#line 52

#line 52
  __nesc_result = MSP430ADC12M$ADCSingle$bind(3U, settings);
#line 52

#line 52
  return __nesc_result;
#line 52
}
#line 52
inline static result_t ADC8IO14P$ADC2$bind(MSP430ADC12Settings_t settings){
#line 52
  unsigned char __nesc_result;
#line 52

#line 52
  __nesc_result = MSP430ADC12M$ADCSingle$bind(2U, settings);
#line 52

#line 52
  return __nesc_result;
#line 52
}
#line 52
inline static result_t ADC8IO14P$ADC1$bind(MSP430ADC12Settings_t settings){
#line 52
  unsigned char __nesc_result;
#line 52

#line 52
  __nesc_result = MSP430ADC12M$ADCSingle$bind(1U, settings);
#line 52

#line 52
  return __nesc_result;
#line 52
}
#line 52
inline static result_t ADC8IO14P$ADC0$bind(MSP430ADC12Settings_t settings){
#line 52
  unsigned char __nesc_result;
#line 52

#line 52
  __nesc_result = MSP430ADC12M$ADCSingle$bind(0U, settings);
#line 52

#line 52
  return __nesc_result;
#line 52
}
#line 52
# 70 "ADC8IO14P.nc"
static inline result_t ADC8IO14P$StdControl$start(void )
#line 70
{

  ADC8IO14P$ADC0$bind(int2adcSettings(((((((((((((((uint32_t )SAMPLE_HOLD_256_CYCLES << 4) + INPUT_CHANNEL_A2) << 3) + SHT_CLOCK_DIV_8) << 3) + REFERENCE_AVcc_AVss) << 2) + SAMPCON_CLOCK_DIV_4) << 2) + SAMPCON_SOURCE_SMCLK) << 2) + SHT_SOURCE_ADC12OSC) << 1) + REFVOLT_LEVEL_2_5));







  ADC8IO14P$ADC1$bind(int2adcSettings(((((((((((((((uint32_t )SAMPLE_HOLD_256_CYCLES << 4) + INPUT_CHANNEL_A0) << 3) + SHT_CLOCK_DIV_8) << 3) + REFERENCE_AVcc_AVss) << 2) + SAMPCON_CLOCK_DIV_4) << 2) + SAMPCON_SOURCE_SMCLK) << 2) + SHT_SOURCE_ADC12OSC) << 1) + REFVOLT_LEVEL_2_5));







  ADC8IO14P$ADC2$bind(int2adcSettings(((((((((((((((uint32_t )SAMPLE_HOLD_256_CYCLES << 4) + INPUT_CHANNEL_A6) << 3) + SHT_CLOCK_DIV_8) << 3) + REFERENCE_AVcc_AVss) << 2) + SAMPCON_CLOCK_DIV_4) << 2) + SAMPCON_SOURCE_SMCLK) << 2) + SHT_SOURCE_ADC12OSC) << 1) + REFVOLT_LEVEL_2_5));







  ADC8IO14P$ADC3$bind(int2adcSettings(((((((((((((((uint32_t )SAMPLE_HOLD_256_CYCLES << 4) + INPUT_CHANNEL_A4) << 3) + SHT_CLOCK_DIV_8) << 3) + REFERENCE_AVcc_AVss) << 2) + SAMPCON_CLOCK_DIV_4) << 2) + SAMPCON_SOURCE_SMCLK) << 2) + SHT_SOURCE_ADC12OSC) << 1) + REFVOLT_LEVEL_2_5));







  ADC8IO14P$ADC4$bind(int2adcSettings(((((((((((((((uint32_t )SAMPLE_HOLD_256_CYCLES << 4) + INPUT_CHANNEL_A5) << 3) + SHT_CLOCK_DIV_8) << 3) + REFERENCE_AVcc_AVss) << 2) + SAMPCON_CLOCK_DIV_4) << 2) + SAMPCON_SOURCE_SMCLK) << 2) + SHT_SOURCE_ADC12OSC) << 1) + REFVOLT_LEVEL_2_5));







  ADC8IO14P$ADC5$bind(int2adcSettings(((((((((((((((uint32_t )SAMPLE_HOLD_256_CYCLES << 4) + INPUT_CHANNEL_A7) << 3) + SHT_CLOCK_DIV_8) << 3) + REFERENCE_AVcc_AVss) << 2) + SAMPCON_CLOCK_DIV_4) << 2) + SAMPCON_SOURCE_SMCLK) << 2) + SHT_SOURCE_ADC12OSC) << 1) + REFVOLT_LEVEL_2_5));







  ADC8IO14P$ADC6$bind(int2adcSettings(((((((((((((((uint32_t )SAMPLE_HOLD_256_CYCLES << 4) + INPUT_CHANNEL_A1) << 3) + SHT_CLOCK_DIV_8) << 3) + REFERENCE_AVcc_AVss) << 2) + SAMPCON_CLOCK_DIV_4) << 2) + SAMPCON_SOURCE_SMCLK) << 2) + SHT_SOURCE_ADC12OSC) << 1) + REFVOLT_LEVEL_2_5));







  ADC8IO14P$ADC7$bind(int2adcSettings(((((((((((((((uint32_t )SAMPLE_HOLD_256_CYCLES << 4) + INPUT_CHANNEL_A3) << 3) + SHT_CLOCK_DIV_8) << 3) + REFERENCE_AVcc_AVss) << 2) + SAMPCON_CLOCK_DIV_4) << 2) + SAMPCON_SOURCE_SMCLK) << 2) + SHT_SOURCE_ADC12OSC) << 1) + REFVOLT_LEVEL_2_5));








  ADC8IO14P$Ctrl0$makeOutput();
  ADC8IO14P$Ctrl1$makeOutput();
  ADC8IO14P$Ctrl2$makeOutput();
  ADC8IO14P$Ctrl3$makeOutput();
  ADC8IO14P$Ctrl4$makeOutput();
  ADC8IO14P$Ctrl5$makeOutput();
  ADC8IO14P$Ctrl6$makeOutput();
  ADC8IO14P$Ctrl7$makeOutput();
  ADC8IO14P$LedUsr$makeOutput();

  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
    {
      ADC8IO14P$countCtrl = 0;
    }
#line 150
    __nesc_atomic_end(__nesc_atomic); }

  ADC8IO14P$Timer$startPeriodic(50);
  return SUCCESS;
}

# 70 "/Users/jingyuancheng/tinyos/moteiv/tinyos-1.x/tos/interfaces/StdControl.nc"
inline static result_t MainP$StdControl$start(void ){
#line 70
  unsigned char __nesc_result;
#line 70

#line 70
  __nesc_result = ADC8IO14P$StdControl$start();
#line 70
  __nesc_result = rcombine(__nesc_result, MSP430ADC12M$StdControl$start());
#line 70

#line 70
  return __nesc_result;
#line 70
}
#line 70
# 66 "/Users/jingyuancheng/tinyos/moteiv/tos/lib/timer/VirtualizeTimerC.nc"
static inline result_t /*HalTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC$0$Init$start(void )
#line 66
{
#line 66
  return SUCCESS;
}

# 48 "/Users/jingyuancheng/tinyos/moteiv/tos/platform/msp430/timer/MSP430AlarmC.nc"
static inline result_t /*HalTimerMilliC.AlarmMilliC.MSP430Alarm*/MSP430AlarmC$0$Init$start(void )
#line 48
{
#line 48
  return SUCCESS;
}

# 30 "/Users/jingyuancheng/tinyos/moteiv/tinyos-1.x/tos/platform/msp430/MSP430GeneralIO.h"
static inline uint8_t TOSH_READ_PORT12_PIN()
#line 30
{
#line 30
  static volatile uint8_t r __asm ("0x0020");

#line 30
  return r & (1 << 2);
}

# 109 "/Users/jingyuancheng/tinyos/moteiv/tinyos-1.x/tos/platform/msp430/MSP430GeneralIOM.nc"
static inline bool MSP430GeneralIOM$Port12$get(void )
#line 109
{
#line 109
  return TOSH_READ_PORT12_PIN() != 0;
}

# 31 "/Users/jingyuancheng/tinyos/moteiv/tinyos-1.x/tos/platform/msp430/MSP430GeneralIO.nc"
inline static bool UartPresenceM$Pin$get(void ){
#line 31
  unsigned char __nesc_result;
#line 31

#line 31
  __nesc_result = MSP430GeneralIOM$Port12$get();
#line 31

#line 31
  return __nesc_result;
#line 31
}
#line 31
# 51 "/Users/jingyuancheng/tinyos/moteiv/tos/platform/tmote/util/uartdetect/UartPresenceM.nc"
static inline bool UartPresenceM$Presence$isConnected(void )
#line 51
{
  return UartPresenceM$Pin$get();
}

# 23 "/Users/jingyuancheng/tinyos/moteiv/tos/interfaces/Detect.nc"
inline static bool FramerP$Detect$isConnected(void ){
#line 23
  unsigned char __nesc_result;
#line 23

#line 23
  __nesc_result = UartPresenceM$Presence$isConnected();
#line 23

#line 23
  return __nesc_result;
#line 23
}
#line 23
# 391 "/Users/jingyuancheng/tinyos/moteiv/tos/platform/msp430/HPLUSART1M.nc"
static inline result_t HPLUSART1M$USARTControl$enableTxIntr(void )
#line 391
{
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 392
    {
      HPLUSART1M$IFG2 &= ~(1 << 5);
      IE2 |= 1 << 5;
    }
#line 395
    __nesc_atomic_end(__nesc_atomic); }
  return SUCCESS;
}

# 175 "/Users/jingyuancheng/tinyos/moteiv/tinyos-1.x/tos/platform/msp430/HPLUSARTControl.nc"
inline static result_t HPLUARTM$USARTControl$enableTxIntr(void ){
#line 175
  unsigned char __nesc_result;
#line 175

#line 175
  __nesc_result = HPLUSART1M$USARTControl$enableTxIntr();
#line 175

#line 175
  return __nesc_result;
#line 175
}
#line 175
# 383 "/Users/jingyuancheng/tinyos/moteiv/tos/platform/msp430/HPLUSART1M.nc"
static inline result_t HPLUSART1M$USARTControl$enableRxIntr(void )
#line 383
{
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 384
    {
      HPLUSART1M$IFG2 &= ~(1 << 4);
      IE2 |= 1 << 4;
    }
#line 387
    __nesc_atomic_end(__nesc_atomic); }
  return SUCCESS;
}

# 174 "/Users/jingyuancheng/tinyos/moteiv/tinyos-1.x/tos/platform/msp430/HPLUSARTControl.nc"
inline static result_t HPLUARTM$USARTControl$enableRxIntr(void ){
#line 174
  unsigned char __nesc_result;
#line 174

#line 174
  __nesc_result = HPLUSART1M$USARTControl$enableRxIntr();
#line 174

#line 174
  return __nesc_result;
#line 174
}
#line 174
# 340 "/Users/jingyuancheng/tinyos/moteiv/tos/platform/msp430/HPLUSART1M.nc"
static inline void HPLUSART1M$USARTControl$setClockRate(uint16_t baudrate, uint8_t mctl)
#line 340
{
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 341
    {
      HPLUSART1M$l_br = baudrate;
      HPLUSART1M$l_mctl = mctl;
      U1BR0 = baudrate & 0x0FF;
      U1BR1 = (baudrate >> 8) & 0x0FF;
      U1MCTL = mctl;
    }
#line 347
    __nesc_atomic_end(__nesc_atomic); }
}

# 169 "/Users/jingyuancheng/tinyos/moteiv/tinyos-1.x/tos/platform/msp430/HPLUSARTControl.nc"
inline static void HPLUARTM$USARTControl$setClockRate(uint16_t baudrate, uint8_t mctl){
#line 169
  HPLUSART1M$USARTControl$setClockRate(baudrate, mctl);
#line 169
}
#line 169
# 332 "/Users/jingyuancheng/tinyos/moteiv/tos/platform/msp430/HPLUSART1M.nc"
static inline void HPLUSART1M$USARTControl$setClockSource(uint8_t source)
#line 332
{
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 333
    {
      HPLUSART1M$l_ssel = source | 0x80;
      HPLUSART1M$U1TCTL &= ~(((0x00 | 0x10) | 0x20) | 0x30);
      HPLUSART1M$U1TCTL |= HPLUSART1M$l_ssel & 0x7F;
    }
#line 337
    __nesc_atomic_end(__nesc_atomic); }
}

# 167 "/Users/jingyuancheng/tinyos/moteiv/tinyos-1.x/tos/platform/msp430/HPLUSARTControl.nc"
inline static void HPLUARTM$USARTControl$setClockSource(uint8_t source){
#line 167
  HPLUSART1M$USARTControl$setClockSource(source);
#line 167
}
#line 167
# 238 "/Users/jingyuancheng/tinyos/moteiv/tos/platform/msp430/HPLUSART1M.nc"
static inline void HPLUSART1M$setUARTModeCommon(void )
#line 238
{
  /* atomic removed: atomic calls only */
#line 239
  {
    U1CTL = 0x01;
    U1CTL |= 0x10;

    U1RCTL &= ~0x08;

    U1CTL = 0x01;
    U1CTL |= 0x10;

    if (HPLUSART1M$l_ssel & 0x80) {
        HPLUSART1M$U1TCTL &= ~(((0x00 | 0x10) | 0x20) | 0x30);
        HPLUSART1M$U1TCTL |= HPLUSART1M$l_ssel & 0x7F;
      }
    else {
        HPLUSART1M$U1TCTL &= ~(((0x00 | 0x10) | 0x20) | 0x30);
        HPLUSART1M$U1TCTL |= 0x10;
      }

    if (HPLUSART1M$l_mctl != 0 || HPLUSART1M$l_br != 0) {
        U1BR0 = HPLUSART1M$l_br & 0x0FF;
        U1BR1 = (HPLUSART1M$l_br >> 8) & 0x0FF;
        U1MCTL = HPLUSART1M$l_mctl;
      }
    else {
        U1BR0 = 0x03;
        U1BR1 = 0x00;
        U1MCTL = 0x4A;
      }

    HPLUSART1M$ME2 &= ~(1 << 4);
    HPLUSART1M$ME2 |= (1 << 5) | (1 << 4);

    U1CTL &= ~0x01;

    HPLUSART1M$IFG2 &= ~((1 << 5) | (1 << 4));
    IE2 &= ~((1 << 5) | (1 << 4));
  }
}

#line 144
static inline void HPLUSART1M$USARTControl$disableUART(void )
#line 144
{
  HPLUSART1M$ME2 &= ~((1 << 5) | (1 << 4));
  TOSH_SEL_UTXD1_IOFUNC();
  TOSH_SEL_URXD1_IOFUNC();
}

# 45 "/Users/jingyuancheng/tinyos/moteiv/tos/platform/tmote/hardware.h"
static inline void TOSH_SEL_UCLK1_IOFUNC()
#line 45
{
#line 45
  static volatile uint8_t r __asm ("0x0033");

#line 45
  r &= ~(1 << 3);
}

#line 46
static inline void TOSH_SEL_SOMI1_IOFUNC()
#line 46
{
#line 46
  static volatile uint8_t r __asm ("0x0033");

#line 46
  r &= ~(1 << 2);
}

#line 47
static inline void TOSH_SEL_SIMO1_IOFUNC()
#line 47
{
#line 47
  static volatile uint8_t r __asm ("0x0033");

#line 47
  r &= ~(1 << 1);
}

# 177 "/Users/jingyuancheng/tinyos/moteiv/tos/platform/msp430/HPLUSART1M.nc"
static inline void HPLUSART1M$USARTControl$disableSPI(void )
#line 177
{
  HPLUSART1M$ME2 &= ~(1 << 4);
  TOSH_SEL_SIMO1_IOFUNC();
  TOSH_SEL_SOMI1_IOFUNC();
  TOSH_SEL_UCLK1_IOFUNC();
}


static inline void HPLUSART1M$USARTControl$disableI2C(void )
#line 185
{
}

#line 314
static inline void HPLUSART1M$USARTControl$setModeUART(void )
#line 314
{
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 315
    {



      HPLUSART1M$USARTControl$disableI2C();
      U1CTL = 0x01;

      HPLUSART1M$USARTControl$disableSPI();
      HPLUSART1M$USARTControl$disableUART();

      TOSH_SEL_UTXD1_MODFUNC();
      TOSH_SEL_URXD1_MODFUNC();

      HPLUSART1M$setUARTModeCommon();
    }
#line 329
    __nesc_atomic_end(__nesc_atomic); }
}

# 153 "/Users/jingyuancheng/tinyos/moteiv/tinyos-1.x/tos/platform/msp430/HPLUSARTControl.nc"
inline static void HPLUARTM$USARTControl$setModeUART(void ){
#line 153
  HPLUSART1M$USARTControl$setModeUART();
#line 153
}
#line 153
# 50 "/Users/jingyuancheng/tinyos/moteiv/tinyos-1.x/tos/platform/msp430/HPLUARTM.nc"
static inline result_t HPLUARTM$UART$init(void )
#line 50
{

  HPLUARTM$USARTControl$setModeUART();
#line 67
  HPLUARTM$USARTControl$setClockSource(0x20);
  HPLUARTM$USARTControl$setClockRate(UBR_SMCLK_115200, UMCTL_SMCLK_115200);








  HPLUARTM$USARTControl$enableRxIntr();
  HPLUARTM$USARTControl$enableTxIntr();
  return SUCCESS;
}

# 62 "/Users/jingyuancheng/tinyos/moteiv/tinyos-1.x/tos/interfaces/HPLUART.nc"
inline static result_t UARTM$HPLUART$init(void ){
#line 62
  unsigned char __nesc_result;
#line 62

#line 62
  __nesc_result = HPLUARTM$UART$init();
#line 62

#line 62
  return __nesc_result;
#line 62
}
#line 62
# 68 "/Users/jingyuancheng/tinyos/moteiv/tinyos-1.x/tos/system/UARTM.nc"
static inline result_t UARTM$Control$start(void )
#line 68
{
  return UARTM$HPLUART$init();
}

# 70 "/Users/jingyuancheng/tinyos/moteiv/tinyos-1.x/tos/interfaces/StdControl.nc"
inline static result_t FramerP$ByteControl$start(void ){
#line 70
  unsigned char __nesc_result;
#line 70

#line 70
  __nesc_result = UARTM$Control$start();
#line 70

#line 70
  return __nesc_result;
#line 70
}
#line 70
# 312 "/Users/jingyuancheng/tinyos/moteiv/tos/platform/tmote/FramerP.nc"
static inline result_t FramerP$StdControl$start(void )
#line 312
{
  result_t status;

#line 314
  FramerP$HDLCInitialize();
  status = FramerP$ByteControl$start();
  if (FramerP$Detect$isConnected() == FALSE) {
      FramerP$USARTControl$disableUARTRx();
    }
  else 
#line 318
    {
      FramerP$USARTControl$enableUARTRx();
    }

  FramerP$USARTControl$disableUARTTx();

  return status;
}

# 120 "/Users/jingyuancheng/tinyos/moteiv/tos/platform/msp430/MSP430InterruptM.nc"
static inline void MSP430InterruptM$Port12$enable(void )
#line 120
{
#line 120
  MSP430InterruptM$P1IE |= 1 << 2;
}

# 34 "/Users/jingyuancheng/tinyos/moteiv/tos/platform/msp430/MSP430Interrupt.nc"
inline static void UartPresenceM$Interrupt$enable(void ){
#line 34
  MSP430InterruptM$Port12$enable();
#line 34
}
#line 34
# 237 "/Users/jingyuancheng/tinyos/moteiv/tos/platform/msp430/MSP430InterruptM.nc"
static inline void MSP430InterruptM$Port12$edge(bool l2h)
#line 237
{
  /* atomic removed: atomic calls only */
#line 238
  {
    if (l2h) {
#line 239
      P1IES &= ~(1 << 2);
      }
    else {
#line 240
      P1IES |= 1 << 2;
      }
  }
}

# 58 "/Users/jingyuancheng/tinyos/moteiv/tos/platform/msp430/MSP430Interrupt.nc"
inline static void UartPresenceM$Interrupt$edge(bool low_to_high){
#line 58
  MSP430InterruptM$Port12$edge(low_to_high);
#line 58
}
#line 58
# 182 "/Users/jingyuancheng/tinyos/moteiv/tos/platform/msp430/MSP430InterruptM.nc"
static inline void MSP430InterruptM$Port12$clear(void )
#line 182
{
#line 182
  MSP430InterruptM$P1IFG &= ~(1 << 2);
}

# 44 "/Users/jingyuancheng/tinyos/moteiv/tos/platform/msp430/MSP430Interrupt.nc"
inline static void UartPresenceM$Interrupt$clear(void ){
#line 44
  MSP430InterruptM$Port12$clear();
#line 44
}
#line 44
# 151 "/Users/jingyuancheng/tinyos/moteiv/tos/platform/msp430/MSP430InterruptM.nc"
static inline void MSP430InterruptM$Port12$disable(void )
#line 151
{
#line 151
  MSP430InterruptM$P1IE &= ~(1 << 2);
}

# 39 "/Users/jingyuancheng/tinyos/moteiv/tos/platform/msp430/MSP430Interrupt.nc"
inline static void UartPresenceM$Interrupt$disable(void ){
#line 39
  MSP430InterruptM$Port12$disable();
#line 39
}
#line 39
# 58 "/Users/jingyuancheng/tinyos/moteiv/tos/platform/tmote/util/uartdetect/UartPresenceM.nc"
static inline result_t UartPresenceM$StdControl$start(void )
#line 58
{
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 59
    {
      UartPresenceM$Interrupt$disable();
      UartPresenceM$Interrupt$clear();

      if (UartPresenceM$Pin$get()) {
          UartPresenceM$m_flags &= ~UartPresenceM$FLAG_EDGE;
          UartPresenceM$taskConnected$postTask();
        }
      else {
        UartPresenceM$m_flags |= UartPresenceM$FLAG_EDGE;
        }
      UartPresenceM$Interrupt$edge(UartPresenceM$m_flags & UartPresenceM$FLAG_EDGE);
      UartPresenceM$Interrupt$enable();
    }
#line 72
    __nesc_atomic_end(__nesc_atomic); }
  return SUCCESS;
}

# 77 "/Users/jingyuancheng/tinyos/moteiv/tinyos-1.x/tos/interfaces/SplitControl.nc"
inline static result_t CC2420AlwaysOnM$RadioControl$start(void ){
#line 77
  unsigned char __nesc_result;
#line 77

#line 77
  __nesc_result = CC2420RadioM$SplitControl$start();
#line 77

#line 77
  return __nesc_result;
#line 77
}
#line 77
# 58 "/Users/jingyuancheng/tinyos/moteiv/tos/lib/sp/cc2420/CC2420AlwaysOnM.nc"
static inline result_t CC2420AlwaysOnM$StdControl$start(void )
#line 58
{
  return CC2420AlwaysOnM$RadioControl$start();
}

# 48 "/Users/jingyuancheng/tinyos/moteiv/tos/platform/msp430/timer/MSP430AlarmC.nc"
static inline result_t /*CC2420SyncAlwaysOnC.AlarmC.MSP430Alarm*/MSP430AlarmC$2$Init$start(void )
#line 48
{
#line 48
  return SUCCESS;
}

# 78 "/Users/jingyuancheng/tinyos/moteiv/tos/lib/timer/VirtualizeAlarmC.nc"
static inline /*CC2420SyncAlwaysOnC.DualAlarmC*/VirtualizeAlarmC$0$error_t /*CC2420SyncAlwaysOnC.DualAlarmC*/VirtualizeAlarmC$0$Init$start(void )
#line 78
{
  return SUCCESS;
}

# 85 "/Users/jingyuancheng/tinyos/moteiv/tos/lib/sched/MainP.nc"
static inline result_t MainP$MainStdControl$default$start(uint8_t id)
#line 85
{
  return FAIL;
}

# 70 "/Users/jingyuancheng/tinyos/moteiv/tinyos-1.x/tos/interfaces/StdControl.nc"
inline static result_t MainP$MainStdControl$start(uint8_t arg_0x101d29020){
#line 70
  unsigned char __nesc_result;
#line 70

#line 70
  switch (arg_0x101d29020) {
#line 70
    case /*MainTimerMilliC.MainControlC*/MainControlC$0$ID:
#line 70
      __nesc_result = /*HalTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC$0$Init$start();
#line 70
      __nesc_result = rcombine(__nesc_result, /*HalTimerMilliC.AlarmMilliC.MSP430Alarm*/MSP430AlarmC$0$Init$start());
#line 70
      break;
#line 70
    case /*MainUartPacketC.MainControlC*/MainControlC$1$ID:
#line 70
      __nesc_result = FramerP$StdControl$start();
#line 70
      break;
#line 70
    case /*MainUartPresenceC.MainControlC*/MainControlC$2$ID:
#line 70
      __nesc_result = UartPresenceM$StdControl$start();
#line 70
      break;
#line 70
    case /*MainSPC.MainControlC*/MainControlC$3$ID:
#line 70
      __nesc_result = /*CC2420SyncAlwaysOnC.DualAlarmC*/VirtualizeAlarmC$0$Init$start();
#line 70
      __nesc_result = rcombine(__nesc_result, CC2420AlwaysOnM$StdControl$start());
#line 70
      __nesc_result = rcombine(__nesc_result, /*CC2420SyncAlwaysOnC.AlarmC.MSP430Alarm*/MSP430AlarmC$2$Init$start());
#line 70
      break;
#line 70
    default:
#line 70
      __nesc_result = MainP$MainStdControl$default$start(arg_0x101d29020);
#line 70
      break;
#line 70
    }
#line 70

#line 70
  return __nesc_result;
#line 70
}
#line 70
# 70 "/Users/jingyuancheng/tinyos/moteiv/tos/lib/sched/MainP.nc"
static inline result_t MainP$MainSplitControl$default$start(uint8_t id)
#line 70
{
  MainP$MainStdControl$start(id);
  return FAIL;
}

# 77 "/Users/jingyuancheng/tinyos/moteiv/tinyos-1.x/tos/interfaces/SplitControl.nc"
inline static result_t MainP$MainSplitControl$start(uint8_t arg_0x101d2caa8){
#line 77
  unsigned char __nesc_result;
#line 77

#line 77
    __nesc_result = MainP$MainSplitControl$default$start(arg_0x101d2caa8);
#line 77

#line 77
  return __nesc_result;
#line 77
}
#line 77
# 27 "/Users/jingyuancheng/tinyos/moteiv/tos/lib/sched/MainP.nc"
static inline void MainP$startDone(uint8_t started)
#line 27
{
  if (started == MainP$m_starting) {

      do {
          MainP$m_starting++;
          if (MainP$MainSplitControl$start(MainP$m_starting) == SUCCESS) {
            return;
            }
        }
      while (
#line 34
      MainP$m_starting < 255);

      MainP$StdControl$start();
      MainP$Boot$booted();
    }
}

# 90 "/Users/jingyuancheng/tinyos/moteiv/tos/platform/msp430/adc/MSP430ADC12M.nc"
static inline void MSP430ADC12M$configureAdcPin(uint8_t inputChannel)
{
  if (inputChannel <= 7) {
      P6SEL |= 1 << inputChannel;
      P6DIR &= ~(1 << inputChannel);
    }
}

# 69 "/Users/jingyuancheng/tinyos/moteiv/tos/platform/msp430/crc.h"
static inline uint16_t crcByte(uint16_t fcs, uint8_t c)
#line 69
{
  return ccitt_crc16_table[(fcs >> 8) ^ c] ^ (fcs << 8);
}

# 49 "/Users/jingyuancheng/tinyos/moteiv/tos/lib/sched/TaskBasic.nc"
inline static result_t FramerP$PacketRcvd$postTask(void ){
#line 49
  unsigned char __nesc_result;
#line 49

#line 49
  __nesc_result = SchedulerBasicP$TaskBasic$postTask(FramerP$PacketRcvd);
#line 49

#line 49
  return __nesc_result;
#line 49
}
#line 49
# 374 "/Users/jingyuancheng/tinyos/moteiv/tos/platform/tmote/FramerP.nc"
static inline result_t FramerP$ByteComm$rxByteReady(uint8_t data, bool error, uint16_t strength)
#line 374
{

  switch (FramerP$gRxState) {

      case FramerP$RXSTATE_NOSYNC: 
        if (data == FramerP$HDLC_FLAG_BYTE && FramerP$gMsgRcvTbl[FramerP$gRxHeadIndex].Length == 0) {
            FramerP$gMsgRcvTbl[FramerP$gRxHeadIndex].Token = 0;
            FramerP$gRxByteCnt = FramerP$gRxRunningCRC = 0;
            FramerP$gpRxBuf = (uint8_t *)FramerP$gMsgRcvTbl[FramerP$gRxHeadIndex].pMsg;
            FramerP$gRxState = FramerP$RXSTATE_PROTO;
          }
      break;

      case FramerP$RXSTATE_PROTO: 
        if (data == FramerP$HDLC_FLAG_BYTE) {
            break;
          }
      FramerP$gMsgRcvTbl[FramerP$gRxHeadIndex].Proto = data;
      FramerP$gRxRunningCRC = crcByte(FramerP$gRxRunningCRC, data);
      switch (data) {
          case FramerP$PROTO_PACKET_ACK: 
            FramerP$gRxState = FramerP$RXSTATE_TOKEN;
          break;
          case FramerP$PROTO_PACKET_NOACK: 
            FramerP$gRxState = FramerP$RXSTATE_INFO;
          break;
          default: 
            FramerP$gRxState = FramerP$RXSTATE_NOSYNC;
          break;
        }
      break;

      case FramerP$RXSTATE_TOKEN: 
        if (data == FramerP$HDLC_FLAG_BYTE) {
            FramerP$gRxState = FramerP$RXSTATE_NOSYNC;
          }
        else {
#line 410
          if (data == FramerP$HDLC_CTLESC_BYTE) {
              FramerP$gMsgRcvTbl[FramerP$gRxHeadIndex].Token = 0x20;
            }
          else {
              FramerP$gMsgRcvTbl[FramerP$gRxHeadIndex].Token ^= data;
              FramerP$gRxRunningCRC = crcByte(FramerP$gRxRunningCRC, FramerP$gMsgRcvTbl[FramerP$gRxHeadIndex].Token);
              FramerP$gRxState = FramerP$RXSTATE_INFO;
            }
          }
#line 418
      break;


      case FramerP$RXSTATE_INFO: 
        if (FramerP$gRxByteCnt > FramerP$HDLC_MTU) {
            FramerP$gRxByteCnt = FramerP$gRxRunningCRC = 0;
            FramerP$gMsgRcvTbl[FramerP$gRxHeadIndex].Length = 0;
            FramerP$gMsgRcvTbl[FramerP$gRxHeadIndex].Token = 0;
            FramerP$gRxState = FramerP$RXSTATE_NOSYNC;
          }
        else {
#line 428
          if (data == FramerP$HDLC_CTLESC_BYTE) {
              FramerP$gRxState = FramerP$RXSTATE_ESC;
            }
          else {
#line 431
            if (data == FramerP$HDLC_FLAG_BYTE) {
                if (FramerP$gRxByteCnt >= 2) {
                    uint16_t usRcvdCRC = FramerP$gpRxBuf[FramerP$gRxByteCnt - 1] & 0xff;

#line 434
                    usRcvdCRC = (usRcvdCRC << 8) | (FramerP$gpRxBuf[FramerP$gRxByteCnt - 2] & 0xff);
                    if (usRcvdCRC == FramerP$gRxRunningCRC) {
                        FramerP$gMsgRcvTbl[FramerP$gRxHeadIndex].Length = FramerP$gRxByteCnt - 2;
                        FramerP$PacketRcvd$postTask();
                        FramerP$gRxHeadIndex++;
#line 438
                        FramerP$gRxHeadIndex %= FramerP$HDLC_QUEUESIZE;
                      }
                    else {
                        FramerP$gMsgRcvTbl[FramerP$gRxHeadIndex].Length = 0;
                        FramerP$gMsgRcvTbl[FramerP$gRxHeadIndex].Token = 0;
                      }
                    if (FramerP$gMsgRcvTbl[FramerP$gRxHeadIndex].Length == 0) {
                        FramerP$gpRxBuf = (uint8_t *)FramerP$gMsgRcvTbl[FramerP$gRxHeadIndex].pMsg;
                        FramerP$gRxState = FramerP$RXSTATE_PROTO;
                      }
                    else {
                        FramerP$gRxState = FramerP$RXSTATE_NOSYNC;
                      }
                  }
                else {
                    FramerP$gMsgRcvTbl[FramerP$gRxHeadIndex].Length = 0;
                    FramerP$gMsgRcvTbl[FramerP$gRxHeadIndex].Token = 0;
                    FramerP$gRxState = FramerP$RXSTATE_NOSYNC;
                  }
                FramerP$gRxByteCnt = FramerP$gRxRunningCRC = 0;
              }
            else {
                FramerP$gpRxBuf[FramerP$gRxByteCnt] = data;
                if (FramerP$gRxByteCnt >= 2) {
                    FramerP$gRxRunningCRC = crcByte(FramerP$gRxRunningCRC, FramerP$gpRxBuf[FramerP$gRxByteCnt - 2]);
                  }
                FramerP$gRxByteCnt++;
              }
            }
          }
#line 466
      break;

      case FramerP$RXSTATE_ESC: 
        if (data == FramerP$HDLC_FLAG_BYTE) {

            FramerP$gRxByteCnt = FramerP$gRxRunningCRC = 0;
            FramerP$gMsgRcvTbl[FramerP$gRxHeadIndex].Length = 0;
            FramerP$gMsgRcvTbl[FramerP$gRxHeadIndex].Token = 0;
            FramerP$gRxState = FramerP$RXSTATE_NOSYNC;
          }
        else {
            data = data ^ 0x20;
            FramerP$gpRxBuf[FramerP$gRxByteCnt] = data;
            if (FramerP$gRxByteCnt >= 2) {
                FramerP$gRxRunningCRC = crcByte(FramerP$gRxRunningCRC, FramerP$gpRxBuf[FramerP$gRxByteCnt - 2]);
              }
            FramerP$gRxByteCnt++;
            FramerP$gRxState = FramerP$RXSTATE_INFO;
          }
      break;

      default: 
        FramerP$gRxState = FramerP$RXSTATE_NOSYNC;
      break;
    }

  return SUCCESS;
}

# 66 "/Users/jingyuancheng/tinyos/moteiv/tinyos-1.x/tos/interfaces/ByteComm.nc"
inline static result_t UARTM$ByteComm$rxByteReady(uint8_t data, bool error, uint16_t strength){
#line 66
  unsigned char __nesc_result;
#line 66

#line 66
  __nesc_result = FramerP$ByteComm$rxByteReady(data, error, strength);
#line 66

#line 66
  return __nesc_result;
#line 66
}
#line 66
# 77 "/Users/jingyuancheng/tinyos/moteiv/tinyos-1.x/tos/system/UARTM.nc"
static inline result_t UARTM$HPLUART$get(uint8_t data)
#line 77
{




  UARTM$ByteComm$rxByteReady(data, FALSE, 0);
  {
  }
#line 83
  ;
  return SUCCESS;
}

# 88 "/Users/jingyuancheng/tinyos/moteiv/tinyos-1.x/tos/interfaces/HPLUART.nc"
inline static result_t HPLUARTM$UART$get(uint8_t data){
#line 88
  unsigned char __nesc_result;
#line 88

#line 88
  __nesc_result = UARTM$HPLUART$get(data);
#line 88

#line 88
  return __nesc_result;
#line 88
}
#line 88
# 90 "/Users/jingyuancheng/tinyos/moteiv/tinyos-1.x/tos/platform/msp430/HPLUARTM.nc"
static inline result_t HPLUARTM$USARTData$rxDone(uint8_t b)
#line 90
{
  return HPLUARTM$UART$get(b);
}

# 53 "/Users/jingyuancheng/tinyos/moteiv/tinyos-1.x/tos/platform/msp430/HPLUSARTFeedback.nc"
inline static result_t HPLUSART1M$USARTData$rxDone(uint8_t data){
#line 53
  unsigned char __nesc_result;
#line 53

#line 53
  __nesc_result = HPLUARTM$USARTData$rxDone(data);
#line 53

#line 53
  return __nesc_result;
#line 53
}
#line 53
# 55 "/Users/jingyuancheng/tinyos/moteiv/tinyos-1.x/tos/interfaces/ByteComm.nc"
inline static result_t FramerP$ByteComm$txByte(uint8_t data){
#line 55
  unsigned char __nesc_result;
#line 55

#line 55
  __nesc_result = UARTM$ByteComm$txByte(data);
#line 55

#line 55
  return __nesc_result;
#line 55
}
#line 55
# 508 "/Users/jingyuancheng/tinyos/moteiv/tos/platform/tmote/FramerP.nc"
static inline result_t FramerP$ByteComm$txByteReady(bool LastByteSuccess)
#line 508
{
  result_t TxResult = SUCCESS;
  uint8_t nextByte;

  if (LastByteSuccess != TRUE) {
      /* atomic removed: atomic calls only */
#line 513
      FramerP$gTxState = FramerP$TXSTATE_ERROR;
      FramerP$PacketSent$postTask();
      return SUCCESS;
    }
  /* atomic removed: atomic calls only */
#line 517
  {
    switch (FramerP$gTxState) {

        case FramerP$TXSTATE_PROTO: 
          FramerP$gTxState = FramerP$TXSTATE_INFO;
        FramerP$gTxRunningCRC = crcByte(FramerP$gTxRunningCRC, (uint8_t )(FramerP$gTxProto & 0x0FF));
        TxResult = FramerP$ByteComm$txByte((uint8_t )(FramerP$gTxProto & 0x0FF));
        break;

        case FramerP$TXSTATE_INFO: 
          nextByte = FramerP$gpTxBuf[FramerP$gTxByteCnt];

        FramerP$gTxRunningCRC = crcByte(FramerP$gTxRunningCRC, nextByte);
        FramerP$gTxByteCnt++;
        if (FramerP$gTxByteCnt >= FramerP$gTxLength) {
            FramerP$gTxState = FramerP$TXSTATE_FCS1;
          }

        TxResult = FramerP$TxArbitraryByte(nextByte);
        break;

        case FramerP$TXSTATE_ESC: 

          FramerP$gTxState = FramerP$gPrevTxState;
        TxResult = FramerP$ByteComm$txByte(FramerP$gTxEscByte ^ 0x20);
        break;

        case FramerP$TXSTATE_FCS1: 
          nextByte = (uint8_t )(FramerP$gTxRunningCRC & 0xff);
        FramerP$gTxState = FramerP$TXSTATE_FCS2;
        TxResult = FramerP$TxArbitraryByte(nextByte);
        break;

        case FramerP$TXSTATE_FCS2: 
          nextByte = (uint8_t )((FramerP$gTxRunningCRC >> 8) & 0xff);
        FramerP$gTxState = FramerP$TXSTATE_ENDFLAG;
        TxResult = FramerP$TxArbitraryByte(nextByte);
        break;

        case FramerP$TXSTATE_ENDFLAG: 

          FramerP$gTxState = FramerP$TXSTATE_PROTO1;
        TxResult = FramerP$ByteComm$txByte(FramerP$HDLC_FLAG_BYTE);
        break;
        case FramerP$TXSTATE_PROTO1: 

          FramerP$gTxState = FramerP$TXSTATE_FINISH;
        TxResult = FramerP$ByteComm$txByte(FramerP$HDLC_FLAG_BYTE);

        break;

        case FramerP$TXSTATE_FINISH: 
          case FramerP$TXSTATE_ERROR: 

            default: 
              break;
      }
  }

  if (TxResult != SUCCESS) {
      FramerP$gTxState = FramerP$TXSTATE_ERROR;
      FramerP$PacketSent$postTask();
    }

  return SUCCESS;
}

# 75 "/Users/jingyuancheng/tinyos/moteiv/tinyos-1.x/tos/interfaces/ByteComm.nc"
inline static result_t UARTM$ByteComm$txByteReady(bool success){
#line 75
  unsigned char __nesc_result;
#line 75

#line 75
  __nesc_result = FramerP$ByteComm$txByteReady(success);
#line 75

#line 75
  return __nesc_result;
#line 75
}
#line 75
# 584 "/Users/jingyuancheng/tinyos/moteiv/tos/platform/tmote/FramerP.nc"
static inline result_t FramerP$ByteComm$txDone(void )
#line 584
{

  if (FramerP$gTxState == FramerP$TXSTATE_FINISH) {
      FramerP$gTxState = FramerP$TXSTATE_IDLE;
      FramerP$PacketSent$postTask();
    }

  return SUCCESS;
}

# 83 "/Users/jingyuancheng/tinyos/moteiv/tinyos-1.x/tos/interfaces/ByteComm.nc"
inline static result_t UARTM$ByteComm$txDone(void ){
#line 83
  unsigned char __nesc_result;
#line 83

#line 83
  __nesc_result = FramerP$ByteComm$txDone();
#line 83

#line 83
  return __nesc_result;
#line 83
}
#line 83
# 87 "/Users/jingyuancheng/tinyos/moteiv/tinyos-1.x/tos/system/UARTM.nc"
static inline result_t UARTM$HPLUART$putDone(void )
#line 87
{
  bool oldState;

  /* atomic removed: atomic calls only */
#line 90
  {
    {
    }
#line 91
    ;
    oldState = UARTM$state;
    UARTM$state = FALSE;
  }








  if (oldState) {
      UARTM$ByteComm$txDone();
      UARTM$ByteComm$txByteReady(TRUE);
    }
  return SUCCESS;
}

# 96 "/Users/jingyuancheng/tinyos/moteiv/tinyos-1.x/tos/interfaces/HPLUART.nc"
inline static result_t HPLUARTM$UART$putDone(void ){
#line 96
  unsigned char __nesc_result;
#line 96

#line 96
  __nesc_result = UARTM$HPLUART$putDone();
#line 96

#line 96
  return __nesc_result;
#line 96
}
#line 96
# 94 "/Users/jingyuancheng/tinyos/moteiv/tinyos-1.x/tos/platform/msp430/HPLUARTM.nc"
static inline result_t HPLUARTM$USARTData$txDone(void )
#line 94
{
  return HPLUARTM$UART$putDone();
}

# 46 "/Users/jingyuancheng/tinyos/moteiv/tinyos-1.x/tos/platform/msp430/HPLUSARTFeedback.nc"
inline static result_t HPLUSART1M$USARTData$txDone(void ){
#line 46
  unsigned char __nesc_result;
#line 46

#line 46
  __nesc_result = HPLUARTM$USARTData$txDone();
#line 46

#line 46
  return __nesc_result;
#line 46
}
#line 46
# 180 "/Users/jingyuancheng/tinyos/moteiv/tos/platform/msp430/MSP430InterruptM.nc"
static inline void MSP430InterruptM$Port10$clear(void )
#line 180
{
#line 180
  MSP430InterruptM$P1IFG &= ~(1 << 0);
}

# 44 "/Users/jingyuancheng/tinyos/moteiv/tos/platform/msp430/MSP430Interrupt.nc"
inline static void HPLCC2420InterruptM$FIFOPInterrupt$clear(void ){
#line 44
  MSP430InterruptM$Port10$clear();
#line 44
}
#line 44
# 149 "/Users/jingyuancheng/tinyos/moteiv/tos/platform/msp430/MSP430InterruptM.nc"
static inline void MSP430InterruptM$Port10$disable(void )
#line 149
{
#line 149
  MSP430InterruptM$P1IE &= ~(1 << 0);
}

# 39 "/Users/jingyuancheng/tinyos/moteiv/tos/platform/msp430/MSP430Interrupt.nc"
inline static void HPLCC2420InterruptM$FIFOPInterrupt$disable(void ){
#line 39
  MSP430InterruptM$Port10$disable();
#line 39
}
#line 39
# 787 "/Users/jingyuancheng/tinyos/moteiv/tos/lib/CC2420Radio/CC2420RadioM.nc"
static inline result_t CC2420RadioM$FIFOP$fired(void )
#line 787
{

  if (!TOSH_READ_CC_FIFO_PIN()) {
      CC2420RadioM$flushRXFIFO(RESOURCE_NONE);
    }
  return SUCCESS;
}

# 51 "/Users/jingyuancheng/tinyos/moteiv/tinyos-1.x/tos/lib/CC2420Radio/HPLCC2420Interrupt.nc"
inline static result_t HPLCC2420InterruptM$FIFOP$fired(void ){
#line 51
  unsigned char __nesc_result;
#line 51

#line 51
  __nesc_result = CC2420RadioM$FIFOP$fired();
#line 51

#line 51
  return __nesc_result;
#line 51
}
#line 51
# 97 "/Users/jingyuancheng/tinyos/moteiv/tos/platform/tmote/HPLCC2420InterruptM.nc"
static inline void HPLCC2420InterruptM$FIFOPInterrupt$fired(void )
#line 97
{
  result_t val = SUCCESS;

#line 99
  HPLCC2420InterruptM$FIFOPInterrupt$clear();
  val = HPLCC2420InterruptM$FIFOP$fired();
  if (val == FAIL) {
      HPLCC2420InterruptM$FIFOPInterrupt$disable();
      HPLCC2420InterruptM$FIFOPInterrupt$clear();
    }
}

# 63 "/Users/jingyuancheng/tinyos/moteiv/tos/platform/msp430/MSP430Interrupt.nc"
inline static void MSP430InterruptM$Port10$fired(void ){
#line 63
  HPLCC2420InterruptM$FIFOPInterrupt$fired();
#line 63
}
#line 63
# 181 "/Users/jingyuancheng/tinyos/moteiv/tos/platform/msp430/MSP430InterruptM.nc"
static inline void MSP430InterruptM$Port11$clear(void )
#line 181
{
#line 181
  MSP430InterruptM$P1IFG &= ~(1 << 1);
}

#line 97
static inline void MSP430InterruptM$Port11$default$fired(void )
#line 97
{
#line 97
  MSP430InterruptM$Port11$clear();
}

# 63 "/Users/jingyuancheng/tinyos/moteiv/tos/platform/msp430/MSP430Interrupt.nc"
inline static void MSP430InterruptM$Port11$fired(void ){
#line 63
  MSP430InterruptM$Port11$default$fired();
#line 63
}
#line 63
# 49 "/Users/jingyuancheng/tinyos/moteiv/tos/lib/sched/TaskBasic.nc"
inline static result_t UartPresenceM$taskDisconnected$postTask(void ){
#line 49
  unsigned char __nesc_result;
#line 49

#line 49
  __nesc_result = SchedulerBasicP$TaskBasic$postTask(UartPresenceM$taskDisconnected);
#line 49

#line 49
  return __nesc_result;
#line 49
}
#line 49
# 79 "/Users/jingyuancheng/tinyos/moteiv/tos/platform/tmote/util/uartdetect/UartPresenceM.nc"
static inline void UartPresenceM$Interrupt$fired(void )
#line 79
{
  UartPresenceM$Interrupt$disable();
  UartPresenceM$Interrupt$clear();


  if (UartPresenceM$m_flags & UartPresenceM$FLAG_EDGE) {
      UartPresenceM$taskConnected$postTask();
    }
  else 
    {
      UartPresenceM$taskDisconnected$postTask();
    }

  UartPresenceM$m_flags ^= UartPresenceM$FLAG_EDGE;
  UartPresenceM$Interrupt$edge(UartPresenceM$m_flags & UartPresenceM$FLAG_EDGE);
  UartPresenceM$Interrupt$enable();
}

# 63 "/Users/jingyuancheng/tinyos/moteiv/tos/platform/msp430/MSP430Interrupt.nc"
inline static void MSP430InterruptM$Port12$fired(void ){
#line 63
  UartPresenceM$Interrupt$fired();
#line 63
}
#line 63
# 183 "/Users/jingyuancheng/tinyos/moteiv/tos/platform/msp430/MSP430InterruptM.nc"
static inline void MSP430InterruptM$Port13$clear(void )
#line 183
{
#line 183
  MSP430InterruptM$P1IFG &= ~(1 << 3);
}

# 44 "/Users/jingyuancheng/tinyos/moteiv/tos/platform/msp430/MSP430Interrupt.nc"
inline static void HPLCC2420InterruptM$FIFOInterrupt$clear(void ){
#line 44
  MSP430InterruptM$Port13$clear();
#line 44
}
#line 44
# 152 "/Users/jingyuancheng/tinyos/moteiv/tos/platform/msp430/MSP430InterruptM.nc"
static inline void MSP430InterruptM$Port13$disable(void )
#line 152
{
#line 152
  MSP430InterruptM$P1IE &= ~(1 << 3);
}

# 39 "/Users/jingyuancheng/tinyos/moteiv/tos/platform/msp430/MSP430Interrupt.nc"
inline static void HPLCC2420InterruptM$FIFOInterrupt$disable(void ){
#line 39
  MSP430InterruptM$Port13$disable();
#line 39
}
#line 39
# 148 "/Users/jingyuancheng/tinyos/moteiv/tos/platform/tmote/HPLCC2420InterruptM.nc"
static inline result_t HPLCC2420InterruptM$FIFO$default$fired(void )
#line 148
{
#line 148
  return FAIL;
}

# 51 "/Users/jingyuancheng/tinyos/moteiv/tinyos-1.x/tos/lib/CC2420Radio/HPLCC2420Interrupt.nc"
inline static result_t HPLCC2420InterruptM$FIFO$fired(void ){
#line 51
  unsigned char __nesc_result;
#line 51

#line 51
  __nesc_result = HPLCC2420InterruptM$FIFO$default$fired();
#line 51

#line 51
  return __nesc_result;
#line 51
}
#line 51
# 138 "/Users/jingyuancheng/tinyos/moteiv/tos/platform/tmote/HPLCC2420InterruptM.nc"
static inline void HPLCC2420InterruptM$FIFOInterrupt$fired(void )
#line 138
{
  result_t val = SUCCESS;

#line 140
  HPLCC2420InterruptM$FIFOInterrupt$clear();
  val = HPLCC2420InterruptM$FIFO$fired();
  if (val == FAIL) {
      HPLCC2420InterruptM$FIFOInterrupt$disable();
      HPLCC2420InterruptM$FIFOInterrupt$clear();
    }
}

# 63 "/Users/jingyuancheng/tinyos/moteiv/tos/platform/msp430/MSP430Interrupt.nc"
inline static void MSP430InterruptM$Port13$fired(void ){
#line 63
  HPLCC2420InterruptM$FIFOInterrupt$fired();
#line 63
}
#line 63
# 63 "/Users/jingyuancheng/tinyos/moteiv/tos/lib/resource/ResourceCmd.nc"
inline static void CC2420ControlM$CmdCCAFired$deferRequest(void ){
#line 63
  /*MSP430ArbiterUSART0C.ArbiterC.ArbiterP*/FcfsArbiterP$1$ResourceCmd$deferRequest(/*CC2420RadioC.CmdCCAFiredC.ResourceC.ResourceUSARTP*/MSP430ResourceUSART0P$0$ID);
#line 63
}
#line 63
# 506 "/Users/jingyuancheng/tinyos/moteiv/tos/lib/CC2420Radio/CC2420ControlM.nc"
static inline result_t CC2420ControlM$CCA$fired(void )
#line 506
{
  CC2420ControlM$CmdCCAFired$deferRequest();
  return FAIL;
}

# 51 "/Users/jingyuancheng/tinyos/moteiv/tinyos-1.x/tos/lib/CC2420Radio/HPLCC2420Interrupt.nc"
inline static result_t HPLCC2420InterruptM$CCA$fired(void ){
#line 51
  unsigned char __nesc_result;
#line 51

#line 51
  __nesc_result = CC2420ControlM$CCA$fired();
#line 51

#line 51
  return __nesc_result;
#line 51
}
#line 51
# 179 "/Users/jingyuancheng/tinyos/moteiv/tos/platform/tmote/HPLCC2420InterruptM.nc"
static inline void HPLCC2420InterruptM$CCAInterrupt$fired(void )
#line 179
{
  result_t val = SUCCESS;

#line 181
  HPLCC2420InterruptM$CCAInterrupt$clear();
  val = HPLCC2420InterruptM$CCA$fired();
  if (val == FAIL) {
      HPLCC2420InterruptM$CCAInterrupt$disable();
      HPLCC2420InterruptM$CCAInterrupt$clear();
    }
}

# 63 "/Users/jingyuancheng/tinyos/moteiv/tos/platform/msp430/MSP430Interrupt.nc"
inline static void MSP430InterruptM$Port14$fired(void ){
#line 63
  HPLCC2420InterruptM$CCAInterrupt$fired();
#line 63
}
#line 63
# 185 "/Users/jingyuancheng/tinyos/moteiv/tos/platform/msp430/MSP430InterruptM.nc"
static inline void MSP430InterruptM$Port15$clear(void )
#line 185
{
#line 185
  MSP430InterruptM$P1IFG &= ~(1 << 5);
}

#line 101
static inline void MSP430InterruptM$Port15$default$fired(void )
#line 101
{
#line 101
  MSP430InterruptM$Port15$clear();
}

# 63 "/Users/jingyuancheng/tinyos/moteiv/tos/platform/msp430/MSP430Interrupt.nc"
inline static void MSP430InterruptM$Port15$fired(void ){
#line 63
  MSP430InterruptM$Port15$default$fired();
#line 63
}
#line 63
# 186 "/Users/jingyuancheng/tinyos/moteiv/tos/platform/msp430/MSP430InterruptM.nc"
static inline void MSP430InterruptM$Port16$clear(void )
#line 186
{
#line 186
  MSP430InterruptM$P1IFG &= ~(1 << 6);
}

#line 102
static inline void MSP430InterruptM$Port16$default$fired(void )
#line 102
{
#line 102
  MSP430InterruptM$Port16$clear();
}

# 63 "/Users/jingyuancheng/tinyos/moteiv/tos/platform/msp430/MSP430Interrupt.nc"
inline static void MSP430InterruptM$Port16$fired(void ){
#line 63
  MSP430InterruptM$Port16$default$fired();
#line 63
}
#line 63
# 187 "/Users/jingyuancheng/tinyos/moteiv/tos/platform/msp430/MSP430InterruptM.nc"
static inline void MSP430InterruptM$Port17$clear(void )
#line 187
{
#line 187
  MSP430InterruptM$P1IFG &= ~(1 << 7);
}

#line 103
static inline void MSP430InterruptM$Port17$default$fired(void )
#line 103
{
#line 103
  MSP430InterruptM$Port17$clear();
}

# 63 "/Users/jingyuancheng/tinyos/moteiv/tos/platform/msp430/MSP430Interrupt.nc"
inline static void MSP430InterruptM$Port17$fired(void ){
#line 63
  MSP430InterruptM$Port17$default$fired();
#line 63
}
#line 63
# 189 "/Users/jingyuancheng/tinyos/moteiv/tos/platform/msp430/MSP430InterruptM.nc"
static inline void MSP430InterruptM$Port20$clear(void )
#line 189
{
#line 189
  MSP430InterruptM$P2IFG &= ~(1 << 0);
}

#line 105
static inline void MSP430InterruptM$Port20$default$fired(void )
#line 105
{
#line 105
  MSP430InterruptM$Port20$clear();
}

# 63 "/Users/jingyuancheng/tinyos/moteiv/tos/platform/msp430/MSP430Interrupt.nc"
inline static void MSP430InterruptM$Port20$fired(void ){
#line 63
  MSP430InterruptM$Port20$default$fired();
#line 63
}
#line 63
# 190 "/Users/jingyuancheng/tinyos/moteiv/tos/platform/msp430/MSP430InterruptM.nc"
static inline void MSP430InterruptM$Port21$clear(void )
#line 190
{
#line 190
  MSP430InterruptM$P2IFG &= ~(1 << 1);
}

#line 106
static inline void MSP430InterruptM$Port21$default$fired(void )
#line 106
{
#line 106
  MSP430InterruptM$Port21$clear();
}

# 63 "/Users/jingyuancheng/tinyos/moteiv/tos/platform/msp430/MSP430Interrupt.nc"
inline static void MSP430InterruptM$Port21$fired(void ){
#line 63
  MSP430InterruptM$Port21$default$fired();
#line 63
}
#line 63
# 191 "/Users/jingyuancheng/tinyos/moteiv/tos/platform/msp430/MSP430InterruptM.nc"
static inline void MSP430InterruptM$Port22$clear(void )
#line 191
{
#line 191
  MSP430InterruptM$P2IFG &= ~(1 << 2);
}

#line 107
static inline void MSP430InterruptM$Port22$default$fired(void )
#line 107
{
#line 107
  MSP430InterruptM$Port22$clear();
}

# 63 "/Users/jingyuancheng/tinyos/moteiv/tos/platform/msp430/MSP430Interrupt.nc"
inline static void MSP430InterruptM$Port22$fired(void ){
#line 63
  MSP430InterruptM$Port22$default$fired();
#line 63
}
#line 63
# 192 "/Users/jingyuancheng/tinyos/moteiv/tos/platform/msp430/MSP430InterruptM.nc"
static inline void MSP430InterruptM$Port23$clear(void )
#line 192
{
#line 192
  MSP430InterruptM$P2IFG &= ~(1 << 3);
}

#line 108
static inline void MSP430InterruptM$Port23$default$fired(void )
#line 108
{
#line 108
  MSP430InterruptM$Port23$clear();
}

# 63 "/Users/jingyuancheng/tinyos/moteiv/tos/platform/msp430/MSP430Interrupt.nc"
inline static void MSP430InterruptM$Port23$fired(void ){
#line 63
  MSP430InterruptM$Port23$default$fired();
#line 63
}
#line 63
# 193 "/Users/jingyuancheng/tinyos/moteiv/tos/platform/msp430/MSP430InterruptM.nc"
static inline void MSP430InterruptM$Port24$clear(void )
#line 193
{
#line 193
  MSP430InterruptM$P2IFG &= ~(1 << 4);
}

#line 109
static inline void MSP430InterruptM$Port24$default$fired(void )
#line 109
{
#line 109
  MSP430InterruptM$Port24$clear();
}

# 63 "/Users/jingyuancheng/tinyos/moteiv/tos/platform/msp430/MSP430Interrupt.nc"
inline static void MSP430InterruptM$Port24$fired(void ){
#line 63
  MSP430InterruptM$Port24$default$fired();
#line 63
}
#line 63
# 194 "/Users/jingyuancheng/tinyos/moteiv/tos/platform/msp430/MSP430InterruptM.nc"
static inline void MSP430InterruptM$Port25$clear(void )
#line 194
{
#line 194
  MSP430InterruptM$P2IFG &= ~(1 << 5);
}

#line 110
static inline void MSP430InterruptM$Port25$default$fired(void )
#line 110
{
#line 110
  MSP430InterruptM$Port25$clear();
}

# 63 "/Users/jingyuancheng/tinyos/moteiv/tos/platform/msp430/MSP430Interrupt.nc"
inline static void MSP430InterruptM$Port25$fired(void ){
#line 63
  MSP430InterruptM$Port25$default$fired();
#line 63
}
#line 63
# 195 "/Users/jingyuancheng/tinyos/moteiv/tos/platform/msp430/MSP430InterruptM.nc"
static inline void MSP430InterruptM$Port26$clear(void )
#line 195
{
#line 195
  MSP430InterruptM$P2IFG &= ~(1 << 6);
}

#line 111
static inline void MSP430InterruptM$Port26$default$fired(void )
#line 111
{
#line 111
  MSP430InterruptM$Port26$clear();
}

# 63 "/Users/jingyuancheng/tinyos/moteiv/tos/platform/msp430/MSP430Interrupt.nc"
inline static void MSP430InterruptM$Port26$fired(void ){
#line 63
  MSP430InterruptM$Port26$default$fired();
#line 63
}
#line 63
# 196 "/Users/jingyuancheng/tinyos/moteiv/tos/platform/msp430/MSP430InterruptM.nc"
static inline void MSP430InterruptM$Port27$clear(void )
#line 196
{
#line 196
  MSP430InterruptM$P2IFG &= ~(1 << 7);
}

#line 112
static inline void MSP430InterruptM$Port27$default$fired(void )
#line 112
{
#line 112
  MSP430InterruptM$Port27$clear();
}

# 63 "/Users/jingyuancheng/tinyos/moteiv/tos/platform/msp430/MSP430Interrupt.nc"
inline static void MSP430InterruptM$Port27$fired(void ){
#line 63
  MSP430InterruptM$Port27$default$fired();
#line 63
}
#line 63
# 198 "/Users/jingyuancheng/tinyos/moteiv/tos/platform/msp430/MSP430InterruptM.nc"
static inline void MSP430InterruptM$NMI$clear(void )
#line 198
{
#line 198
  IFG1 &= ~(1 << 4);
}

#line 114
static inline void MSP430InterruptM$NMI$default$fired(void )
#line 114
{
#line 114
  MSP430InterruptM$NMI$clear();
}

# 63 "/Users/jingyuancheng/tinyos/moteiv/tos/platform/msp430/MSP430Interrupt.nc"
inline static void MSP430InterruptM$NMI$fired(void ){
#line 63
  MSP430InterruptM$NMI$default$fired();
#line 63
}
#line 63
# 199 "/Users/jingyuancheng/tinyos/moteiv/tos/platform/msp430/MSP430InterruptM.nc"
static inline void MSP430InterruptM$OF$clear(void )
#line 199
{
#line 199
  IFG1 &= ~(1 << 1);
}

#line 115
static inline void MSP430InterruptM$OF$default$fired(void )
#line 115
{
#line 115
  MSP430InterruptM$OF$clear();
}

# 63 "/Users/jingyuancheng/tinyos/moteiv/tos/platform/msp430/MSP430Interrupt.nc"
inline static void MSP430InterruptM$OF$fired(void ){
#line 63
  MSP430InterruptM$OF$default$fired();
#line 63
}
#line 63
# 200 "/Users/jingyuancheng/tinyos/moteiv/tos/platform/msp430/MSP430InterruptM.nc"
static inline void MSP430InterruptM$ACCV$clear(void )
#line 200
{
#line 200
  FCTL3 &= ~0x0004;
}

#line 116
static inline void MSP430InterruptM$ACCV$default$fired(void )
#line 116
{
#line 116
  MSP430InterruptM$ACCV$clear();
}

# 63 "/Users/jingyuancheng/tinyos/moteiv/tos/platform/msp430/MSP430Interrupt.nc"
inline static void MSP430InterruptM$ACCV$fired(void ){
#line 63
  MSP430InterruptM$ACCV$default$fired();
#line 63
}
#line 63
# 475 "/Users/jingyuancheng/tinyos/moteiv/tos/platform/msp430/HPLUSART0M.nc"
static inline result_t HPLUSART0M$USARTData$default$rxDone(uint8_t data)
#line 475
{
#line 475
  return SUCCESS;
}

# 53 "/Users/jingyuancheng/tinyos/moteiv/tinyos-1.x/tos/platform/msp430/HPLUSARTFeedback.nc"
inline static result_t HPLUSART0M$USARTData$rxDone(uint8_t data){
#line 53
  unsigned char __nesc_result;
#line 53

#line 53
  __nesc_result = HPLUSART0M$USARTData$default$rxDone(data);
#line 53

#line 53
  return __nesc_result;
#line 53
}
#line 53
# 70 "/Users/jingyuancheng/tinyos/moteiv/tos/platform/msp430/HPLUSART0M.nc"
static inline void HPLUSART0M$HPLI2CInterrupt$default$fired(void )
#line 70
{
}

# 43 "/Users/jingyuancheng/tinyos/moteiv/tinyos-1.x/tos/platform/msp430/HPLI2CInterrupt.nc"
inline static void HPLUSART0M$HPLI2CInterrupt$fired(void ){
#line 43
  HPLUSART0M$HPLI2CInterrupt$default$fired();
#line 43
}
#line 43
# 473 "/Users/jingyuancheng/tinyos/moteiv/tos/platform/msp430/HPLUSART0M.nc"
static inline result_t HPLUSART0M$USARTData$default$txDone(void )
#line 473
{
#line 473
  return SUCCESS;
}

# 46 "/Users/jingyuancheng/tinyos/moteiv/tinyos-1.x/tos/platform/msp430/HPLUSARTFeedback.nc"
inline static result_t HPLUSART0M$USARTData$txDone(void ){
#line 46
  unsigned char __nesc_result;
#line 46

#line 46
  __nesc_result = HPLUSART0M$USARTData$default$txDone();
#line 46

#line 46
  return __nesc_result;
#line 46
}
#line 46
# 529 "/Users/jingyuancheng/tinyos/moteiv/tos/platform/msp430/adc/MSP430ADC12M.nc"
static inline void MSP430ADC12M$HPLADC12$memOverflow(void )
#line 529
{
}

# 214 "/Users/jingyuancheng/tinyos/moteiv/tos/platform/msp430/adc/RefVoltM.nc"
static inline void RefVoltM$HPLADC12$memOverflow(void )
#line 214
{
}

# 61 "/Users/jingyuancheng/tinyos/moteiv/tos/platform/msp430/adc/HPLADC12.nc"
inline static void HPLADC12M$HPLADC12$memOverflow(void ){
#line 61
  RefVoltM$HPLADC12$memOverflow();
#line 61
  MSP430ADC12M$HPLADC12$memOverflow();
#line 61
}
#line 61
# 530 "/Users/jingyuancheng/tinyos/moteiv/tos/platform/msp430/adc/MSP430ADC12M.nc"
static inline void MSP430ADC12M$HPLADC12$timeOverflow(void )
#line 530
{
}

# 215 "/Users/jingyuancheng/tinyos/moteiv/tos/platform/msp430/adc/RefVoltM.nc"
static inline void RefVoltM$HPLADC12$timeOverflow(void )
#line 215
{
}

# 62 "/Users/jingyuancheng/tinyos/moteiv/tos/platform/msp430/adc/HPLADC12.nc"
inline static void HPLADC12M$HPLADC12$timeOverflow(void ){
#line 62
  RefVoltM$HPLADC12$timeOverflow();
#line 62
  MSP430ADC12M$HPLADC12$timeOverflow();
#line 62
}
#line 62
#line 58
inline static void MSP430ADC12M$HPLADC12$resetIFGs(void ){
#line 58
  HPLADC12M$HPLADC12$resetIFGs();
#line 58
}
#line 58
# 443 "/Users/jingyuancheng/tinyos/moteiv/tos/platform/msp430/adc/MSP430ADC12M.nc"
static inline uint16_t *MSP430ADC12M$ADCMultiple$default$dataReady(uint8_t num, uint16_t *buf, 
uint16_t length)
{
  return (uint16_t *)0;
}

# 168 "/Users/jingyuancheng/tinyos/moteiv/tos/platform/msp430/adc/MSP430ADC12Multiple.nc"
inline static uint16_t *MSP430ADC12M$ADCMultiple$dataReady(uint8_t arg_0x102dc6220, uint16_t *buf, uint16_t length){
#line 168
  unsigned int *__nesc_result;
#line 168

#line 168
    __nesc_result = MSP430ADC12M$ADCMultiple$default$dataReady(arg_0x102dc6220, buf, length);
#line 168

#line 168
  return __nesc_result;
#line 168
}
#line 168
# 91 "/Users/jingyuancheng/tinyos/moteiv/tos/platform/msp430/adc/HPLADC12M.nc"
static inline uint16_t HPLADC12M$HPLADC12$getMem(uint8_t i)
#line 91
{
  return *((uint16_t *)(int *)0x0140 + i);
}

# 52 "/Users/jingyuancheng/tinyos/moteiv/tos/platform/msp430/adc/HPLADC12.nc"
inline static uint16_t MSP430ADC12M$HPLADC12$getMem(uint8_t i){
#line 52
  unsigned int __nesc_result;
#line 52

#line 52
  __nesc_result = HPLADC12M$HPLADC12$getMem(i);
#line 52

#line 52
  return __nesc_result;
#line 52
}
#line 52
#line 50
inline static void MSP430ADC12M$HPLADC12$setMemControl(uint8_t index, adc12memctl_t memControl){
#line 50
  HPLADC12M$HPLADC12$setMemControl(index, memControl);
#line 50
}
#line 50
# 81 "/Users/jingyuancheng/tinyos/moteiv/tos/platform/msp430/adc/HPLADC12M.nc"
static inline adc12memctl_t HPLADC12M$HPLADC12$getMemControl(uint8_t i)
#line 81
{
  adc12memctl_t x = { .inch = 0, .sref = 0, .eos = 0 };
  uint8_t *memCtlPtr = (uint8_t *)(char *)0x0080;

#line 84
  if (i < 16) {
      memCtlPtr += i;
      x = * (adc12memctl_t *)memCtlPtr;
    }
  return x;
}

# 51 "/Users/jingyuancheng/tinyos/moteiv/tos/platform/msp430/adc/HPLADC12.nc"
inline static adc12memctl_t MSP430ADC12M$HPLADC12$getMemControl(uint8_t i){
#line 51
  struct __nesc_unnamed4262 __nesc_result;
#line 51

#line 51
  __nesc_result = HPLADC12M$HPLADC12$getMemControl(i);
#line 51

#line 51
  return __nesc_result;
#line 51
}
#line 51
# 475 "/Users/jingyuancheng/tinyos/moteiv/tos/platform/msp430/adc/MSP430ADC12M.nc"
static inline void MSP430ADC12M$HPLADC12$converted(uint8_t number)
#line 475
{
  switch (MSP430ADC12M$cmode) 
    {
      case SINGLE_CHANNEL: 
        {
          volatile uint8_t ownerTmp = MSP430ADC12M$owner;

#line 481
          MSP430ADC12M$stopConversion();
          MSP430ADC12M$ADCSingle$dataReady(ownerTmp, MSP430ADC12M$HPLADC12$getMem(0));
        }
      break;
      case REPEAT_SINGLE_CHANNEL: 
        if (MSP430ADC12M$ADCSingle$dataReady(MSP430ADC12M$owner, MSP430ADC12M$HPLADC12$getMem(0)) == FAIL) {
          MSP430ADC12M$stopConversion();
          }
#line 488
      break;
      case SEQUENCE_OF_CHANNELS: 
        {
          uint16_t i = 0;
#line 491
          uint16_t length = MSP430ADC12M$bufLength - MSP430ADC12M$bufOffset > 16 ? 16 : MSP430ADC12M$bufLength - MSP430ADC12M$bufOffset;

#line 492
          do {
              * MSP430ADC12M$bufPtr++ = MSP430ADC12M$HPLADC12$getMem(i);
            }
          while (
#line 494
          ++i < length);

          MSP430ADC12M$bufOffset += length;

          if (MSP430ADC12M$bufLength - MSP430ADC12M$bufOffset > 15) {
            return;
            }
          else {
#line 500
            if (MSP430ADC12M$bufLength - MSP430ADC12M$bufOffset > 0) {
                adc12memctl_t memctl = MSP430ADC12M$HPLADC12$getMemControl(0);

#line 502
                memctl.eos = 1;
                MSP430ADC12M$HPLADC12$setMemControl(MSP430ADC12M$bufLength - MSP430ADC12M$bufOffset, memctl);
              }
            else 
#line 504
              {
                MSP430ADC12M$stopConversion();
                MSP430ADC12M$ADCMultiple$dataReady(MSP430ADC12M$owner, MSP430ADC12M$bufPtr - MSP430ADC12M$bufLength, MSP430ADC12M$bufLength);
              }
            }
        }
#line 509
      break;
      case REPEAT_SEQUENCE_OF_CHANNELS: 
        {
          uint8_t i = 0;

#line 513
          do {
              * MSP430ADC12M$bufPtr++ = MSP430ADC12M$HPLADC12$getMem(i);
            }
          while (
#line 515
          ++i < MSP430ADC12M$bufLength);
          if ((MSP430ADC12M$bufPtr = MSP430ADC12M$ADCMultiple$dataReady(MSP430ADC12M$owner, MSP430ADC12M$bufPtr - MSP430ADC12M$bufLength, MSP430ADC12M$bufLength)) == 0) {
            MSP430ADC12M$stopConversion();
            }
#line 518
          break;
        }
      default: 
        {

          MSP430ADC12M$HPLADC12$resetIFGs();
        }
      break;
    }
}

# 216 "/Users/jingyuancheng/tinyos/moteiv/tos/platform/msp430/adc/RefVoltM.nc"
static inline void RefVoltM$HPLADC12$converted(uint8_t number)
#line 216
{
}

# 63 "/Users/jingyuancheng/tinyos/moteiv/tos/platform/msp430/adc/HPLADC12.nc"
inline static void HPLADC12M$HPLADC12$converted(uint8_t number){
#line 63
  RefVoltM$HPLADC12$converted(number);
#line 63
  MSP430ADC12M$HPLADC12$converted(number);
#line 63
}
#line 63
# 97 "/Users/jingyuancheng/tinyos/moteiv/tos/platform/msp430/timer/MSP430TimerAExclusiveM.nc"
static inline result_t MSP430TimerAExclusiveM$TimerExclusive$stopTimer(uint8_t rh)
#line 97
{
  if (!MSP430TimerAExclusiveM$ResourceValidate$validateUser(rh)) {
    return FAIL;
    }
  MSP430TimerAExclusiveM$TimerA$setMode(MSP430TIMER_STOP_MODE);
  return SUCCESS;
}

# 22 "/Users/jingyuancheng/tinyos/moteiv/tos/platform/msp430/timer/TimerExclusive.nc"
inline static result_t MSP430ADC12M$TimerExclusive$stopTimer(uint8_t rh){
#line 22
  unsigned char __nesc_result;
#line 22

#line 22
  __nesc_result = MSP430TimerAExclusiveM$TimerExclusive$stopTimer(rh);
#line 22

#line 22
  return __nesc_result;
#line 22
}
#line 22
# 219 "/Users/jingyuancheng/tinyos/moteiv/tos/lib/resource/FcfsArbiterP.nc"
static inline void /*MSP430ArbiterTimerAC.ArbiterC.ArbiterP*/FcfsArbiterP$0$ResourceCmdAsync$release(uint8_t id)
#line 219
{
  /*MSP430ArbiterTimerAC.ArbiterC.ArbiterP*/FcfsArbiterP$0$Resource$release(id);
}

# 85 "/Users/jingyuancheng/tinyos/moteiv/tos/lib/resource/ResourceCmdAsync.nc"
inline static void MSP430ADC12M$Resource$release(void ){
#line 85
  /*MSP430ArbiterTimerAC.ArbiterC.ArbiterP*/FcfsArbiterP$0$ResourceCmdAsync$release(/*MSP430ADC12C.ResourceC*/MSP430ResourceTimerAC$1$ID);
#line 85
}
#line 85
# 117 "/Users/jingyuancheng/tinyos/moteiv/tos/platform/msp430/adc/HPLADC12M.nc"
static inline void HPLADC12M$HPLADC12$stopConversion(void )
#line 117
{
  HPLADC12M$ADC12CTL1 &= ~((1 << 1) | (3 << 1));
  HPLADC12M$ADC12CTL0 &= ~0x0002;
}

# 82 "/Users/jingyuancheng/tinyos/moteiv/tos/platform/msp430/adc/HPLADC12.nc"
inline static void MSP430ADC12M$HPLADC12$stopConversion(void ){
#line 82
  HPLADC12M$HPLADC12$stopConversion();
#line 82
}
#line 82
# 49 "/Users/jingyuancheng/tinyos/moteiv/tos/lib/sched/TaskBasic.nc"
inline static result_t RefVoltM$switchOffDelay$postTask(void ){
#line 49
  unsigned char __nesc_result;
#line 49

#line 49
  __nesc_result = SchedulerBasicP$TaskBasic$postTask(RefVoltM$switchOffDelay);
#line 49

#line 49
  return __nesc_result;
#line 49
}
#line 49
# 147 "/Users/jingyuancheng/tinyos/moteiv/tos/platform/msp430/adc/RefVoltM.nc"
static inline result_t RefVoltM$RefVolt$release(void )
#line 147
{
  result_t result = FAIL;

  /* atomic removed: atomic calls only */
#line 150
  {
    if (RefVoltM$semaCount <= 0) {
      result = FAIL;
      }
    else 
#line 153
      {
        RefVoltM$semaCount--;
        if (RefVoltM$semaCount == 0) {
            if (RefVoltM$state == RefVoltM$REFERENCE_1_5V_PENDING || 
            RefVoltM$state == RefVoltM$REFERENCE_2_5V_PENDING) {
                RefVoltM$switchOff = TRUE;
                RefVoltM$switchRefOff();
              }
            else {
                RefVoltM$switchOff = TRUE;
                RefVoltM$switchOffDelay$postTask();
              }
            result = SUCCESS;
          }
      }
  }
  return result;
}

# 109 "/Users/jingyuancheng/tinyos/moteiv/tos/platform/msp430/adc/RefVolt.nc"
inline static result_t MSP430ADC12M$RefVolt$release(void ){
#line 109
  unsigned char __nesc_result;
#line 109

#line 109
  __nesc_result = RefVoltM$RefVolt$release();
#line 109

#line 109
  return __nesc_result;
#line 109
}
#line 109
# 159 "/Users/jingyuancheng/tinyos/moteiv/tos/platform/msp430/adc/MSP430ADC12M.nc"
static inline result_t MSP430ADC12M$releaseRefVolt(uint8_t num)
{
  if (MSP430ADC12M$adc12settings[num].gotRefVolt == 1) {
      MSP430ADC12M$RefVolt$release();
      MSP430ADC12M$adc12settings[num].gotRefVolt = 0;
      return SUCCESS;
    }
  return FAIL;
}

# 67 "/Users/jingyuancheng/tinyos/moteiv/tos/platform/msp430/adc/MSP430ADC12Single.nc"
inline static msp430ADCresult_t ADC8IO14P$ADC1$getData(void ){
#line 67
  enum __nesc_unnamed4259 __nesc_result;
#line 67

#line 67
  __nesc_result = MSP430ADC12M$ADCSingle$getData(1U);
#line 67

#line 67
  return __nesc_result;
#line 67
}
#line 67
# 231 "ADC8IO14P.nc"
static inline result_t ADC8IO14P$ADC0$dataReady(uint16_t data)
{
  /* atomic removed: atomic calls only */
  {
    ADCMsg_t *body = (ADCMsg_t *)ADC8IO14P$m_msg.data;

#line 236
    body->data[ADC8IO14P$countCtrl * 8] = data >> 4;
    ADC8IO14P$ADC1$getData();
    {
      unsigned char __nesc_temp = 
#line 238
      SUCCESS;

#line 238
      return __nesc_temp;
    }
  }
}

# 67 "/Users/jingyuancheng/tinyos/moteiv/tos/platform/msp430/adc/MSP430ADC12Single.nc"
inline static msp430ADCresult_t ADC8IO14P$ADC2$getData(void ){
#line 67
  enum __nesc_unnamed4259 __nesc_result;
#line 67

#line 67
  __nesc_result = MSP430ADC12M$ADCSingle$getData(2U);
#line 67

#line 67
  return __nesc_result;
#line 67
}
#line 67
# 243 "ADC8IO14P.nc"
static inline result_t ADC8IO14P$ADC1$dataReady(uint16_t data)
{
  /* atomic removed: atomic calls only */
  {
    ADCMsg_t *body = (ADCMsg_t *)ADC8IO14P$m_msg.data;

#line 248
    body->data[ADC8IO14P$countCtrl * 8 + 1] = data >> 4;
    ADC8IO14P$ADC2$getData();
    {
      unsigned char __nesc_temp = 
#line 250
      SUCCESS;

#line 250
      return __nesc_temp;
    }
  }
}

# 67 "/Users/jingyuancheng/tinyos/moteiv/tos/platform/msp430/adc/MSP430ADC12Single.nc"
inline static msp430ADCresult_t ADC8IO14P$ADC3$getData(void ){
#line 67
  enum __nesc_unnamed4259 __nesc_result;
#line 67

#line 67
  __nesc_result = MSP430ADC12M$ADCSingle$getData(3U);
#line 67

#line 67
  return __nesc_result;
#line 67
}
#line 67
# 254 "ADC8IO14P.nc"
static inline result_t ADC8IO14P$ADC2$dataReady(uint16_t data)
{
  /* atomic removed: atomic calls only */
  {
    ADCMsg_t *body = (ADCMsg_t *)ADC8IO14P$m_msg.data;

#line 259
    body->data[ADC8IO14P$countCtrl * 8 + 2] = data >> 4;
    ADC8IO14P$ADC3$getData();
    {
      unsigned char __nesc_temp = 
#line 261
      SUCCESS;

#line 261
      return __nesc_temp;
    }
  }
}

# 67 "/Users/jingyuancheng/tinyos/moteiv/tos/platform/msp430/adc/MSP430ADC12Single.nc"
inline static msp430ADCresult_t ADC8IO14P$ADC4$getData(void ){
#line 67
  enum __nesc_unnamed4259 __nesc_result;
#line 67

#line 67
  __nesc_result = MSP430ADC12M$ADCSingle$getData(4U);
#line 67

#line 67
  return __nesc_result;
#line 67
}
#line 67
# 264 "ADC8IO14P.nc"
static inline result_t ADC8IO14P$ADC3$dataReady(uint16_t data)
{
  /* atomic removed: atomic calls only */
  {
    ADCMsg_t *body = (ADCMsg_t *)ADC8IO14P$m_msg.data;

#line 269
    body->data[ADC8IO14P$countCtrl * 8 + 3] = data >> 4;
    ADC8IO14P$ADC4$getData();
    {
      unsigned char __nesc_temp = 
#line 271
      SUCCESS;

#line 271
      return __nesc_temp;
    }
  }
}

# 67 "/Users/jingyuancheng/tinyos/moteiv/tos/platform/msp430/adc/MSP430ADC12Single.nc"
inline static msp430ADCresult_t ADC8IO14P$ADC5$getData(void ){
#line 67
  enum __nesc_unnamed4259 __nesc_result;
#line 67

#line 67
  __nesc_result = MSP430ADC12M$ADCSingle$getData(5U);
#line 67

#line 67
  return __nesc_result;
#line 67
}
#line 67
# 274 "ADC8IO14P.nc"
static inline result_t ADC8IO14P$ADC4$dataReady(uint16_t data)
{
  /* atomic removed: atomic calls only */
  {
    ADCMsg_t *body = (ADCMsg_t *)ADC8IO14P$m_msg.data;

#line 279
    body->data[ADC8IO14P$countCtrl * 8 + 4] = data >> 4;
    ADC8IO14P$ADC5$getData();
    {
      unsigned char __nesc_temp = 
#line 281
      SUCCESS;

#line 281
      return __nesc_temp;
    }
  }
}

# 67 "/Users/jingyuancheng/tinyos/moteiv/tos/platform/msp430/adc/MSP430ADC12Single.nc"
inline static msp430ADCresult_t ADC8IO14P$ADC6$getData(void ){
#line 67
  enum __nesc_unnamed4259 __nesc_result;
#line 67

#line 67
  __nesc_result = MSP430ADC12M$ADCSingle$getData(6U);
#line 67

#line 67
  return __nesc_result;
#line 67
}
#line 67
# 284 "ADC8IO14P.nc"
static inline result_t ADC8IO14P$ADC5$dataReady(uint16_t data)
{
  /* atomic removed: atomic calls only */
  {
    ADCMsg_t *body = (ADCMsg_t *)ADC8IO14P$m_msg.data;

#line 289
    body->data[ADC8IO14P$countCtrl * 8 + 5] = data >> 4;
    ADC8IO14P$ADC6$getData();
    {
      unsigned char __nesc_temp = 
#line 291
      SUCCESS;

#line 291
      return __nesc_temp;
    }
  }
}

# 67 "/Users/jingyuancheng/tinyos/moteiv/tos/platform/msp430/adc/MSP430ADC12Single.nc"
inline static msp430ADCresult_t ADC8IO14P$ADC7$getData(void ){
#line 67
  enum __nesc_unnamed4259 __nesc_result;
#line 67

#line 67
  __nesc_result = MSP430ADC12M$ADCSingle$getData(7U);
#line 67

#line 67
  return __nesc_result;
#line 67
}
#line 67
# 294 "ADC8IO14P.nc"
static inline result_t ADC8IO14P$ADC6$dataReady(uint16_t data)
{
  /* atomic removed: atomic calls only */
  {
    ADCMsg_t *body = (ADCMsg_t *)ADC8IO14P$m_msg.data;

#line 299
    body->data[ADC8IO14P$countCtrl * 8 + 6] = data >> 4;
    ADC8IO14P$ADC7$getData();
    {
      unsigned char __nesc_temp = 
#line 301
      SUCCESS;

#line 301
      return __nesc_temp;
    }
  }
}

# 49 "/Users/jingyuancheng/tinyos/moteiv/tos/lib/sched/TaskBasic.nc"
inline static result_t ADC8IO14P$taskSendData$postTask(void ){
#line 49
  unsigned char __nesc_result;
#line 49

#line 49
  __nesc_result = SchedulerBasicP$TaskBasic$postTask(ADC8IO14P$taskSendData);
#line 49

#line 49
  return __nesc_result;
#line 49
}
#line 49
# 70 "/Users/jingyuancheng/tinyos/moteiv/tinyos-1.x/tos/platform/msp430/MSP430GeneralIO.h"
static inline void TOSH_CLR_PORT56_PIN()
#line 70
{
#line 70
  static volatile uint8_t r __asm ("0x0031");

#line 70
  r &= ~(1 << 6);
}

# 466 "/Users/jingyuancheng/tinyos/moteiv/tinyos-1.x/tos/platform/msp430/MSP430GeneralIOM.nc"
static inline void MSP430GeneralIOM$Port56$setLow(void )
#line 466
{
#line 466
  TOSH_CLR_PORT56_PIN();
}

# 28 "/Users/jingyuancheng/tinyos/moteiv/tinyos-1.x/tos/platform/msp430/MSP430GeneralIO.nc"
inline static void ADC8IO14P$LedUsr$setLow(void ){
#line 28
  MSP430GeneralIOM$Port56$setLow();
#line 28
}
#line 28
# 305 "ADC8IO14P.nc"
static inline result_t ADC8IO14P$ADC7$dataReady(uint16_t data)
{
  /* atomic removed: atomic calls only */
  {
    ADCMsg_t *body = (ADCMsg_t *)ADC8IO14P$m_msg.data;

#line 310
    body->data[ADC8IO14P$countCtrl * 8 + 7] = data >> 4;
    ADC8IO14P$countCtrl = (ADC8IO14P$countCtrl + 1) % 8;
    ADC8IO14P$switchCtrl();
    if (ADC8IO14P$countCtrl == 0 && ADC8IO14P$m_sending == FALSE) 
      {
        body->count = ADC8IO14P$m_count++;
        body->count = 0xFFFF;

        ADC8IO14P$LedUsr$setLow();
        ADC8IO14P$taskSendData$postTask();
      }
    else 
      {
        ADC8IO14P$ADC0$getData();
      }
  }
  return SUCCESS;
}

# 219 "/Users/jingyuancheng/tinyos/moteiv/tos/platform/msp430/msp430hardware.h"
__nesc_atomic_t __nesc_atomic_start(void )
{
  __nesc_atomic_t result = are_interrupts_enabled();

#line 222
  __nesc_disable_interrupt();
  return result;
}

void __nesc_atomic_end(__nesc_atomic_t reenable_interrupts)
{
  if (reenable_interrupts) {
    __nesc_enable_interrupt();
    }
}

# 11 "/Users/jingyuancheng/tinyos/moteiv/tos/platform/msp430/timer/MSP430TimerCommonM.nc"
 __attribute((wakeup)) __attribute((interrupt(12))) void sig_TIMERA0_VECTOR(void )
#line 11
{
#line 11
  MSP430TimerCommonM$VectorTimerA0$fired();
}

# 33 "/Users/jingyuancheng/tinyos/moteiv/tos/platform/msp430/timer/MSP430Timer.nc"
static void /*MSP430TimerC.MSP430TimerA*/MSP430TimerM$0$Timer$overflow(void ){
#line 33
  MSP430TimerAExclusiveM$TimerA$overflow();
#line 33
  MSP430ResourceConfigTimerAP$TimerA$overflow();
#line 33
  MSP430DCOCalibP$TimerA$overflow();
#line 33
  /*MSP430TimerC.MSP430TimerA0*/MSP430TimerCapComM$0$Timer$overflow();
#line 33
  /*MSP430TimerC.MSP430TimerA1*/MSP430TimerCapComM$1$Timer$overflow();
#line 33
  /*MSP430TimerC.MSP430TimerA2*/MSP430TimerCapComM$2$Timer$overflow();
#line 33
}
#line 33
# 168 "/Users/jingyuancheng/tinyos/moteiv/tos/platform/msp430/timer/MSP430TimerCapComM.nc"
static void /*MSP430TimerC.MSP430TimerA0*/MSP430TimerCapComM$0$Event$fired(void )
{
  if (/*MSP430TimerC.MSP430TimerA0*/MSP430TimerCapComM$0$Control$getControl().cap) {
    /*MSP430TimerC.MSP430TimerA0*/MSP430TimerCapComM$0$Capture$captured(/*MSP430TimerC.MSP430TimerA0*/MSP430TimerCapComM$0$Capture$getEvent());
    }
  else {
#line 173
    /*MSP430TimerC.MSP430TimerA0*/MSP430TimerCapComM$0$Compare$fired();
    }
}

#line 168
static void /*MSP430TimerC.MSP430TimerA1*/MSP430TimerCapComM$1$Event$fired(void )
{
  if (/*MSP430TimerC.MSP430TimerA1*/MSP430TimerCapComM$1$Control$getControl().cap) {
    /*MSP430TimerC.MSP430TimerA1*/MSP430TimerCapComM$1$Capture$captured(/*MSP430TimerC.MSP430TimerA1*/MSP430TimerCapComM$1$Capture$getEvent());
    }
  else {
#line 173
    /*MSP430TimerC.MSP430TimerA1*/MSP430TimerCapComM$1$Compare$fired();
    }
}

#line 168
static void /*MSP430TimerC.MSP430TimerA2*/MSP430TimerCapComM$2$Event$fired(void )
{
  if (/*MSP430TimerC.MSP430TimerA2*/MSP430TimerCapComM$2$Control$getControl().cap) {
    /*MSP430TimerC.MSP430TimerA2*/MSP430TimerCapComM$2$Capture$captured(/*MSP430TimerC.MSP430TimerA2*/MSP430TimerCapComM$2$Capture$getEvent());
    }
  else {
#line 173
    /*MSP430TimerC.MSP430TimerA2*/MSP430TimerCapComM$2$Compare$fired();
    }
}

# 12 "/Users/jingyuancheng/tinyos/moteiv/tos/platform/msp430/timer/MSP430TimerCommonM.nc"
 __attribute((wakeup)) __attribute((interrupt(10))) void sig_TIMERA1_VECTOR(void )
#line 12
{
#line 12
  MSP430TimerCommonM$VectorTimerA1$fired();
}

#line 13
 __attribute((wakeup)) __attribute((interrupt(26))) void sig_TIMERB0_VECTOR(void )
#line 13
{
#line 13
  MSP430TimerCommonM$VectorTimerB0$fired();
}

# 134 "/Users/jingyuancheng/tinyos/moteiv/tos/platform/msp430/timer/MSP430TimerM.nc"
static void /*MSP430TimerC.MSP430TimerB*/MSP430TimerM$1$Event$default$fired(uint8_t n)
{
}

# 4 "/Users/jingyuancheng/tinyos/moteiv/tos/platform/msp430/timer/MSP430TimerEvent.nc"
static void /*MSP430TimerC.MSP430TimerB*/MSP430TimerM$1$Event$fired(uint8_t arg_0x101bf0748){
#line 4
  switch (arg_0x101bf0748) {
#line 4
    case 0:
#line 4
      /*MSP430TimerC.MSP430TimerB0*/MSP430TimerCapComM$3$Event$fired();
#line 4
      break;
#line 4
    case 1:
#line 4
      /*MSP430TimerC.MSP430TimerB1*/MSP430TimerCapComM$4$Event$fired();
#line 4
      break;
#line 4
    case 2:
#line 4
      /*MSP430TimerC.MSP430TimerB2*/MSP430TimerCapComM$5$Event$fired();
#line 4
      break;
#line 4
    case 3:
#line 4
      /*MSP430TimerC.MSP430TimerB3*/MSP430TimerCapComM$6$Event$fired();
#line 4
      break;
#line 4
    case 4:
#line 4
      /*MSP430TimerC.MSP430TimerB4*/MSP430TimerCapComM$7$Event$fired();
#line 4
      break;
#line 4
    case 5:
#line 4
      /*MSP430TimerC.MSP430TimerB5*/MSP430TimerCapComM$8$Event$fired();
#line 4
      break;
#line 4
    case 6:
#line 4
      /*MSP430TimerC.MSP430TimerB6*/MSP430TimerCapComM$9$Event$fired();
#line 4
      break;
#line 4
    case 7:
#line 4
      /*MSP430TimerC.MSP430TimerB*/MSP430TimerM$1$Overflow$fired();
#line 4
      break;
#line 4
    default:
#line 4
      /*MSP430TimerC.MSP430TimerB*/MSP430TimerM$1$Event$default$fired(arg_0x101bf0748);
#line 4
      break;
#line 4
    }
#line 4
}
#line 4
# 183 "/Users/jingyuancheng/tinyos/moteiv/tos/lib/resource/FcfsArbiterP.nc"
static void /*MSP430ArbiterUSART0C.ArbiterC.ArbiterP*/FcfsArbiterP$1$ResourceCmdAsync$urgentRequest(uint8_t id, uint8_t rh)
#line 183
{
  bool grant = TRUE;

#line 185
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 185
    {
      if (/*MSP430ArbiterUSART0C.ArbiterC.ArbiterP*/FcfsArbiterP$1$m_granted == RESOURCE_NONE) {
          /*MSP430ArbiterUSART0C.ArbiterC.ArbiterP*/FcfsArbiterP$1$m_granted = rh = id;
          /*MSP430ArbiterUSART0C.ArbiterC.ArbiterP*/FcfsArbiterP$1$ResourceConfigure$configure(id);
        }
      else {
#line 190
        if (rh != /*MSP430ArbiterUSART0C.ArbiterC.ArbiterP*/FcfsArbiterP$1$m_granted) {
            if (rqueue_pushFront(/*MSP430ArbiterUSART0C.ArbiterC.ArbiterP*/FcfsArbiterP$1$queue(), id)) {
              /*MSP430ArbiterUSART0C.ArbiterC.ArbiterP*/FcfsArbiterP$1$m_urgentCount++;
              }
#line 193
            grant = FALSE;
          }
        }
    }
#line 196
    __nesc_atomic_end(__nesc_atomic); }
#line 196
  if (grant) {
      /*MSP430ArbiterUSART0C.ArbiterC.ArbiterP*/FcfsArbiterP$1$ResourceCmdAsync$granted(id, rh);
    }
  else {
      /*MSP430ArbiterUSART0C.ArbiterC.ArbiterP*/FcfsArbiterP$1$Arbiter$requested();
    }
}

#line 239
static void /*MSP430ArbiterUSART0C.ArbiterC.ArbiterP*/FcfsArbiterP$1$ResourceConfigure$default$configure(uint8_t id)
#line 239
{
}

# 29 "/Users/jingyuancheng/tinyos/moteiv/tos/lib/resource/ResourceConfigure.nc"
static void /*MSP430ArbiterUSART0C.ArbiterC.ArbiterP*/FcfsArbiterP$1$ResourceConfigure$configure(uint8_t arg_0x101b719b0){
#line 29
  switch (arg_0x101b719b0) {
#line 29
    case /*CC2420RadioC.CmdCCAFiredC.ResourceC.ResourceUSARTP*/MSP430ResourceUSART0P$0$ID:
#line 29
      /*MSP430ResourceConfigUSART0C.ConfigUSARTP*/MSP430ResourceConfigUSARTP$0$ConfigSPI$configure();
#line 29
      break;
#line 29
    case /*CC2420RadioC.CmdSplitControlInitC.ResourceC.ResourceUSARTP*/MSP430ResourceUSART0P$1$ID:
#line 29
      /*MSP430ResourceConfigUSART0C.ConfigUSARTP*/MSP430ResourceConfigUSARTP$0$ConfigSPI$configure();
#line 29
      break;
#line 29
    case /*CC2420RadioC.CmdSplitControlStartC.ResourceC.ResourceUSARTP*/MSP430ResourceUSART0P$2$ID:
#line 29
      /*MSP430ResourceConfigUSART0C.ConfigUSARTP*/MSP430ResourceConfigUSARTP$0$ConfigSPI$configure();
#line 29
      break;
#line 29
    case /*CC2420RadioC.CmdSplitControlStopC.ResourceC.ResourceUSARTP*/MSP430ResourceUSART0P$3$ID:
#line 29
      /*MSP430ResourceConfigUSART0C.ConfigUSARTP*/MSP430ResourceConfigUSARTP$0$ConfigSPI$configure();
#line 29
      break;
#line 29
    case /*CC2420RadioC.CmdCmds.ResourceC.ResourceUSARTP*/MSP430ResourceUSART0P$4$ID:
#line 29
      /*MSP430ResourceConfigUSART0C.ConfigUSARTP*/MSP430ResourceConfigUSARTP$0$ConfigSPI$configure();
#line 29
      break;
#line 29
    case /*CC2420RadioC.CmdFlushRXFIFOC.ResourceC.ResourceUSARTP*/MSP430ResourceUSART0P$5$ID:
#line 29
      /*MSP430ResourceConfigUSART0C.ConfigUSARTP*/MSP430ResourceConfigUSARTP$0$ConfigSPI$configure();
#line 29
      break;
#line 29
    case /*CC2420RadioC.CmdReceiveC.ResourceC.ResourceUSARTP*/MSP430ResourceUSART0P$6$ID:
#line 29
      /*MSP430ResourceConfigUSART0C.ConfigUSARTP*/MSP430ResourceConfigUSARTP$0$ConfigSPI$configure();
#line 29
      break;
#line 29
    case /*CC2420RadioC.CmdTransmitC.ResourceC.ResourceUSARTP*/MSP430ResourceUSART0P$7$ID:
#line 29
      /*MSP430ResourceConfigUSART0C.ConfigUSARTP*/MSP430ResourceConfigUSARTP$0$ConfigSPI$configure();
#line 29
      break;
#line 29
    case /*CC2420RadioC.CmdTryToSendC.ResourceC.ResourceUSARTP*/MSP430ResourceUSART0P$8$ID:
#line 29
      /*MSP430ResourceConfigUSART0C.ConfigUSARTP*/MSP430ResourceConfigUSARTP$0$ConfigSPI$configure();
#line 29
      break;
#line 29
    case /*CC2420TimeStampingC.CmdWriteTimeStampC.ResourceC.ResourceUSARTP*/MSP430ResourceUSART0P$9$ID:
#line 29
      /*MSP430ResourceConfigUSART0C.ConfigUSARTP*/MSP430ResourceConfigUSARTP$0$ConfigSPI$configure();
#line 29
      break;
#line 29
    default:
#line 29
      /*MSP430ArbiterUSART0C.ArbiterC.ArbiterP*/FcfsArbiterP$1$ResourceConfigure$default$configure(arg_0x101b719b0);
#line 29
      break;
#line 29
    }
#line 29
}
#line 29
# 49 "/Users/jingyuancheng/tinyos/moteiv/tos/platform/msp430/resource/MSP430ResourceConfigUSARTP.nc"
static void /*MSP430ResourceConfigUSART0C.ConfigUSARTP*/MSP430ResourceConfigUSARTP$0$ConfigSPI$configure(void )
#line 49
{
  /* atomic removed: atomic calls only */
#line 50
  {
    if (/*MSP430ResourceConfigUSART0C.ConfigUSARTP*/MSP430ResourceConfigUSARTP$0$m_mode != /*MSP430ResourceConfigUSART0C.ConfigUSARTP*/MSP430ResourceConfigUSARTP$0$MODE_SPI) {
        /*MSP430ResourceConfigUSART0C.ConfigUSARTP*/MSP430ResourceConfigUSARTP$0$m_mode = /*MSP430ResourceConfigUSART0C.ConfigUSARTP*/MSP430ResourceConfigUSARTP$0$MODE_SPI;
        /*MSP430ResourceConfigUSART0C.ConfigUSARTP*/MSP430ResourceConfigUSARTP$0$HPLUSARTControl$setModeSPI();
        /*MSP430ResourceConfigUSART0C.ConfigUSARTP*/MSP430ResourceConfigUSARTP$0$HPLUSARTControl$disableRxIntr();
        /*MSP430ResourceConfigUSART0C.ConfigUSARTP*/MSP430ResourceConfigUSARTP$0$HPLUSARTControl$disableTxIntr();
      }
  }
}

# 43 "/Users/jingyuancheng/tinyos/moteiv/tos/lib/util/reservedQueue.h"
static bool rqueue_isQueued(ReservedQueue_t *q, uint8_t id)
#line 43
{
  return q->next[id] != RQUEUE_NONE || q->tail == id;
}

# 46 "/Users/jingyuancheng/tinyos/moteiv/tos/lib/sp/cc2420/CC2420TimeStampingM.nc"
static void CC2420TimeStampingM$CmdWriteTimeStamp$granted(uint8_t rh)
#line 46
{
  CC2420TimeStampingM$HPLCC2420RAM$write(rh, CC2420TimeStampingM$TX_FIFO_MSG_START + CC2420TimeStampingM$sendStampOffset, sizeof(uint32_t ), (void *)CC2420TimeStampingM$timestampMsgBuf->data + CC2420TimeStampingM$sendStampOffset);
  CC2420TimeStampingM$sendStampOffset = -1;
  CC2420TimeStampingM$CmdWriteTimeStamp$release();
}

# 251 "/Users/jingyuancheng/tinyos/moteiv/tos/platform/tmote/HPLCC2420M.nc"
static result_t HPLCC2420M$HPLCC2420RAM$write(uint8_t rh, uint16_t addr, uint8_t _length, uint8_t *buffer)
#line 251
{
  uint8_t i = 0;

#line 253
  if (HPLCC2420M$request(rh, HPLCC2420M$BUSY_CMD)) {
      { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 254
        {
          HPLCC2420M$ramaddr = addr;
          HPLCC2420M$ramlen = _length;
          HPLCC2420M$rambuf = buffer;
        }
#line 258
        __nesc_atomic_end(__nesc_atomic); }
      TOSH_CLR_RADIO_CSN_PIN();

      HPLCC2420M$USARTControl$isTxIntrPending();
      HPLCC2420M$USARTControl$rx();
      HPLCC2420M$USARTControl$tx((HPLCC2420M$ramaddr & 0x7F) | 0x80);
      while (HPLCC2420M$f_enabled && !HPLCC2420M$USARTControl$isTxIntrPending()) ;
      HPLCC2420M$USARTControl$tx((HPLCC2420M$ramaddr >> 1) & 0xC0);
      while (HPLCC2420M$f_enabled && !HPLCC2420M$USARTControl$isTxIntrPending()) ;
      for (i = 0; i < HPLCC2420M$ramlen; i++) {
          HPLCC2420M$USARTControl$tx(HPLCC2420M$rambuf[i]);
          while (HPLCC2420M$f_enabled && !HPLCC2420M$USARTControl$isTxIntrPending()) ;
        }
      while (HPLCC2420M$f_enabled && !HPLCC2420M$USARTControl$isTxEmpty()) ;
      TOSH_SET_RADIO_CSN_PIN();
      HPLCC2420M$release();
      return HPLCC2420M$signalRAMWr$postTask();
    }
  return FAIL;
}

#line 116
static bool HPLCC2420M$request(uint8_t rh, uint8_t busy)
#line 116
{
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 117
    {
      if (HPLCC2420M$f_busy == HPLCC2420M$IDLE && HPLCC2420M$CC2420Validate$validateUser(rh)) {
          HPLCC2420M$f_busy = busy;
          {
            unsigned char __nesc_temp = 
#line 120
            TRUE;

            {
#line 120
              __nesc_atomic_end(__nesc_atomic); 
#line 120
              return __nesc_temp;
            }
          }
        }
      else 
#line 122
        {
          {
            unsigned char __nesc_temp = 
#line 123
            FALSE;

            {
#line 123
              __nesc_atomic_end(__nesc_atomic); 
#line 123
              return __nesc_temp;
            }
          }
        }
    }
#line 127
    __nesc_atomic_end(__nesc_atomic); }
}

# 411 "/Users/jingyuancheng/tinyos/moteiv/tos/platform/msp430/HPLUSART0M.nc"
static result_t HPLUSART0M$USARTControl$isTxIntrPending(void )
#line 411
{
  if (HPLUSART0M$IFG1 & (1 << 7)) {
      HPLUSART0M$IFG1 &= ~(1 << 7);
      return SUCCESS;
    }
  return FAIL;
}

#line 465
static uint8_t HPLUSART0M$USARTControl$rx(void )
#line 465
{
  uint8_t value;

#line 467
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 467
    {
      value = U0RXBUF;
    }
#line 469
    __nesc_atomic_end(__nesc_atomic); }
  return value;
}

# 114 "/Users/jingyuancheng/tinyos/moteiv/tos/lib/sched/SchedulerBasicP.nc"
static bool SchedulerBasicP$pushTask(uint8_t id)
#line 114
{
  if (SchedulerBasicP$m_next[id] == SchedulerBasicP$NONE && SchedulerBasicP$m_tail != id) {
      if (SchedulerBasicP$m_head == SchedulerBasicP$NONE) {
          SchedulerBasicP$m_head = id;
          SchedulerBasicP$m_tail = id;
        }
      else {
          SchedulerBasicP$m_next[SchedulerBasicP$m_tail] = id;
          SchedulerBasicP$m_tail = id;
        }
      return TRUE;
    }
  else {
      return FALSE;
    }
}

# 105 "/Users/jingyuancheng/tinyos/moteiv/tos/lib/resource/FcfsArbiterP.nc"
static void /*MSP430ArbiterUSART0C.ArbiterC.ArbiterP*/FcfsArbiterP$1$Resource$release(uint8_t id)
#line 105
{
  uint8_t grantMode = 0;

  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 108
    {
      if (id == /*MSP430ArbiterUSART0C.ArbiterC.ArbiterP*/FcfsArbiterP$1$m_granted) {
          uint8_t granted = rqueue_pop(/*MSP430ArbiterUSART0C.ArbiterC.ArbiterP*/FcfsArbiterP$1$queue());

          if (granted == RQUEUE_NONE) {
              /*MSP430ArbiterUSART0C.ArbiterC.ArbiterP*/FcfsArbiterP$1$m_granted = RESOURCE_NONE;
              grantMode = 2;
            }
          else {
              /*MSP430ArbiterUSART0C.ArbiterC.ArbiterP*/FcfsArbiterP$1$m_granted = granted;
              if (/*MSP430ArbiterUSART0C.ArbiterC.ArbiterP*/FcfsArbiterP$1$m_urgentCount > 0) {
                  /*MSP430ArbiterUSART0C.ArbiterC.ArbiterP*/FcfsArbiterP$1$m_urgentCount--;
                  grantMode = 1;
                }
            }
        }
      else {
          {
#line 125
            __nesc_atomic_end(__nesc_atomic); 
#line 125
            return;
          }
        }
    }
#line 128
    __nesc_atomic_end(__nesc_atomic); }
  if (grantMode == 0) {
#line 129
    /*MSP430ArbiterUSART0C.ArbiterC.ArbiterP*/FcfsArbiterP$1$GrantTask$postTask();
    }
  else {
#line 130
    if (grantMode == 1) {
#line 130
      /*MSP430ArbiterUSART0C.ArbiterC.ArbiterP*/FcfsArbiterP$1$GrantTask$postUrgentTask();
      }
    else {
#line 131
      /*MSP430ArbiterUSART0C.ArbiterC.ArbiterP*/FcfsArbiterP$1$Arbiter$idle();
      }
    }
}

# 48 "/Users/jingyuancheng/tinyos/moteiv/tos/lib/util/reservedQueue.h"
static bool rqueue_remove(ReservedQueue_t *q, uint8_t id)
#line 48
{


  if (id != RQUEUE_NONE && q->head != RQUEUE_NONE) {


      if (id == q->head) {
          q->head = q->next[id];
          if (q->head == RQUEUE_NONE) {
            q->tail = RQUEUE_NONE;
            }
        }
      else {
          uint8_t prev = RQUEUE_NONE;
          uint8_t node = q->head;


          while (node != id) {
              prev = node;
              node = q->next[node];


              if (node == RQUEUE_NONE) {
                return FALSE;
                }
            }

          q->next[prev] = q->next[node];
          if (q->tail == node) {
            q->tail = prev;
            }
        }

      q->next[id] = RQUEUE_NONE;


      return TRUE;
    }


  return FALSE;
}

# 49 "/Users/jingyuancheng/tinyos/moteiv/tos/lib/sched/TaskBasic.nc"
static result_t /*MSP430ArbiterUSART0C.ArbiterC.ArbiterP*/FcfsArbiterP$1$GrantTask$postTask(void ){
#line 49
  unsigned char __nesc_result;
#line 49

#line 49
  __nesc_result = SchedulerBasicP$TaskBasic$postTask(19U);
#line 49

#line 49
  return __nesc_result;
#line 49
}
#line 49
# 131 "/Users/jingyuancheng/tinyos/moteiv/tos/lib/sched/SchedulerBasicP.nc"
static bool SchedulerBasicP$pushFront(uint8_t id)
#line 131
{
  if (!SchedulerBasicP$isQueued(id)) {
      SchedulerBasicP$m_next[id] = SchedulerBasicP$m_head;
      SchedulerBasicP$m_head = id;
      if (SchedulerBasicP$m_tail == SchedulerBasicP$NONE) {
        SchedulerBasicP$m_tail = id;
        }
#line 137
      return TRUE;
    }
  else {
      return FALSE;
    }
}

# 168 "/Users/jingyuancheng/tinyos/moteiv/tos/lib/CC2420Radio/CC2420RadioM.nc"
static void CC2420RadioM$CmdFlushRXFIFO$granted(uint8_t rh)
#line 168
{


  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 171
    {
      CC2420RadioM$FIFOP$disable();
      CC2420RadioM$HPLChipcon$read(rh, 0x3F);
      CC2420RadioM$HPLChipcon$cmd(rh, 0x08);
      CC2420RadioM$HPLChipcon$cmd(rh, 0x08);
      CC2420RadioM$bPacketReceiving = FALSE;
      CC2420RadioM$m_rxFifoCount = 0;
      cqueue_init(&CC2420RadioM$m_timestampQueue, CC2420RadioM$NUM_TIMESTAMPS);
      if (CC2420RadioM$m_sfdReceiving) {
          CC2420RadioM$m_sfdReceiving = FALSE;
          CC2420RadioM$SFD$enableCapture(TRUE);
        }
      CC2420RadioM$FIFOP$startWait(FALSE);
    }
#line 184
    __nesc_atomic_end(__nesc_atomic); }
  CC2420RadioM$CmdFlushRXFIFO$release();
}

# 86 "/Users/jingyuancheng/tinyos/moteiv/tos/platform/tmote/HPLCC2420InterruptM.nc"
static result_t HPLCC2420InterruptM$FIFOP$disable(void )
#line 86
{
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 87
    {
      HPLCC2420InterruptM$FIFOPInterrupt$disable();
      HPLCC2420InterruptM$FIFOPInterrupt$clear();
    }
#line 90
    __nesc_atomic_end(__nesc_atomic); }
  return SUCCESS;
}

# 186 "/Users/jingyuancheng/tinyos/moteiv/tos/platform/tmote/HPLCC2420M.nc"
static uint16_t HPLCC2420M$HPLCC2420$read(uint8_t rh, uint8_t addr)
#line 186
{
  uint16_t data = 0;

#line 188
  if (HPLCC2420M$request(rh, HPLCC2420M$BUSY_CMD)) {
      TOSH_CLR_RADIO_CSN_PIN();

      HPLCC2420M$USARTControl$isTxIntrPending();
      HPLCC2420M$USARTControl$rx();
      HPLCC2420M$USARTControl$tx(addr | 0x40);
      while (HPLCC2420M$f_enabled && !HPLCC2420M$USARTControl$isRxIntrPending()) ;
      HPLCC2420M$USARTControl$rx();
      HPLCC2420M$USARTControl$tx(0);
      while (HPLCC2420M$f_enabled && !HPLCC2420M$USARTControl$isRxIntrPending()) ;
      data = (HPLCC2420M$USARTControl$rx() << 8) & 0xFF00;
      HPLCC2420M$USARTControl$tx(0);
      while (HPLCC2420M$f_enabled && !HPLCC2420M$USARTControl$isRxIntrPending()) ;
      data = data | (HPLCC2420M$USARTControl$rx() & 0x0FF);
      TOSH_SET_RADIO_CSN_PIN();
      HPLCC2420M$release();
    }
  return data;
}

# 426 "/Users/jingyuancheng/tinyos/moteiv/tos/platform/msp430/HPLUSART0M.nc"
static result_t HPLUSART0M$USARTControl$isRxIntrPending(void )
#line 426
{
  if (HPLUSART0M$IFG1 & (1 << 6)) {
      HPLUSART0M$IFG1 &= ~(1 << 6);
      return SUCCESS;
    }
  return FAIL;
}

# 140 "/Users/jingyuancheng/tinyos/moteiv/tos/platform/tmote/HPLCC2420M.nc"
static uint8_t HPLCC2420M$HPLCC2420$cmd(uint8_t rh, uint8_t addr)
#line 140
{
  uint8_t status = 0;

#line 142
  if (HPLCC2420M$request(rh, HPLCC2420M$BUSY_CMD)) {
      TOSH_CLR_RADIO_CSN_PIN();

      HPLCC2420M$USARTControl$isTxIntrPending();
      HPLCC2420M$USARTControl$rx();
      HPLCC2420M$USARTControl$tx(addr);
      while (HPLCC2420M$f_enabled && !HPLCC2420M$USARTControl$isRxIntrPending()) ;
      status = HPLCC2420M$adjustStatusByte(HPLCC2420M$USARTControl$rx());
      TOSH_SET_RADIO_CSN_PIN();
      HPLCC2420M$release();
    }
  return status;
}

# 49 "/Users/jingyuancheng/tinyos/moteiv/tos/lib/util/circularQueue.h"
static void cqueue_init(CircularQueue_t *cq, CircularQueueIndex_t size)
{
  cq->front = size;
  cq->back = size;
  cq->size = size;
}

# 193 "/Users/jingyuancheng/tinyos/moteiv/tos/platform/tmote/HPLCC2420InterruptM.nc"
static result_t HPLCC2420InterruptM$SFD$enableCapture(bool low_to_high)
#line 193
{
  uint8_t _direction;

#line 195
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 195
    {
      TOSH_SEL_CC_SFD_MODFUNC();
      HPLCC2420InterruptM$SFDControl$disableEvents();
      if (low_to_high) {
#line 198
        _direction = MSP430TIMER_CM_RISING;
        }
      else {
#line 199
        _direction = MSP430TIMER_CM_FALLING;
        }
#line 200
      HPLCC2420InterruptM$SFDControl$setControlAsCapture(_direction);
      HPLCC2420InterruptM$SFDCapture$clearOverflow();
      HPLCC2420InterruptM$SFDControl$clearPendingInterrupt();
      HPLCC2420InterruptM$SFDControl$enableEvents();
    }
#line 204
    __nesc_atomic_end(__nesc_atomic); }
  return SUCCESS;
}

#line 73
static result_t HPLCC2420InterruptM$FIFOP$startWait(bool low_to_high)
#line 73
{
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 74
    {
      HPLCC2420InterruptM$FIFOPInterrupt$disable();
      HPLCC2420InterruptM$FIFOPInterrupt$clear();
      HPLCC2420InterruptM$FIFOPInterrupt$edge(low_to_high);
      HPLCC2420InterruptM$FIFOPInterrupt$enable();
    }
#line 79
    __nesc_atomic_end(__nesc_atomic); }
  return SUCCESS;
}

# 511 "/Users/jingyuancheng/tinyos/moteiv/tos/lib/CC2420Radio/CC2420ControlM.nc"
static void CC2420ControlM$CmdCmds$granted(uint8_t rh)
#line 511
{
  CC2420ControlM$cmds_t c;

#line 513
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 513
    {
      c = CC2420ControlM$cmds;
      CC2420ControlM$cmds.byte = 0;
    }
#line 516
    __nesc_atomic_end(__nesc_atomic); }
  if (c.freqselect) {
#line 517
      CC2420ControlM$doCmdFreqSelect(rh);
    }
#line 518
  if (c.setrfpower) {
#line 518
      CC2420ControlM$doCmdSetRFPower(rh);
    }
#line 519
  if (c.setshortaddress) {
#line 519
      CC2420ControlM$doCmdSetShortAddress(rh);
    }
#line 520
  if (c.mdmctrl0) {
#line 520
      CC2420ControlM$doCmdMDMCTRL0(rh);
    }
#line 521
  switch (c.oscillator) {
      case CC2420ControlM$CMD_OSCILLATOR_ON: CC2420ControlM$doCmdOscillatorOn(rh);
#line 522
      break;
      case CC2420ControlM$CMD_OSCILLATOR_OFF: CC2420ControlM$doCmdOscillatorOff(rh);
#line 523
      break;
    }
  switch (c.rxtxmode) {
      case CC2420ControlM$CMD_SRXON: CC2420ControlM$doCmdSRXON(rh);
#line 526
      break;
      case CC2420ControlM$CMD_STXON: CC2420ControlM$doCmdSTXON(rh);
#line 527
      break;
      case CC2420ControlM$CMD_STXONCCA: CC2420ControlM$doCmdSTXONCCA(rh);
#line 528
      break;
    }
  CC2420ControlM$CmdCmds$release();
}

# 161 "/Users/jingyuancheng/tinyos/moteiv/tos/platform/tmote/HPLCC2420M.nc"
static uint8_t HPLCC2420M$HPLCC2420$write(uint8_t rh, uint8_t addr, uint16_t data)
#line 161
{
  uint8_t status = 0;

#line 163
  if (HPLCC2420M$request(rh, HPLCC2420M$BUSY_CMD)) {
      TOSH_CLR_RADIO_CSN_PIN();

      HPLCC2420M$USARTControl$isTxIntrPending();
      HPLCC2420M$USARTControl$rx();
      HPLCC2420M$USARTControl$tx(addr);
      while (HPLCC2420M$f_enabled && !HPLCC2420M$USARTControl$isRxIntrPending()) ;
      status = HPLCC2420M$adjustStatusByte(HPLCC2420M$USARTControl$rx());
      HPLCC2420M$USARTControl$tx((data >> 8) & 0x0FF);
      while (HPLCC2420M$f_enabled && !HPLCC2420M$USARTControl$isTxIntrPending()) ;
      HPLCC2420M$USARTControl$tx(data & 0x0FF);
      while (HPLCC2420M$f_enabled && !HPLCC2420M$USARTControl$isTxEmpty()) ;
      TOSH_SET_RADIO_CSN_PIN();
      HPLCC2420M$release();
    }
  return status;
}

# 99 "/Users/jingyuancheng/tinyos/moteiv/tos/lib/util/reservedQueue.h"
static bool rqueue_push_priv(ReservedQueue_t *q, uint8_t id, bool second)
#line 99
{
  if (rqueue_isQueued(q, id)) {
      return FALSE;
    }

  if (q->head == RQUEUE_NONE) {
      q->head = id;
      q->tail = id;
    }
  else {
#line 108
    if (second) {

        q->next[id] = q->next[q->head];
        q->next[q->head] = id;
        if (q->tail == q->head) {
          q->tail = id;
          }
      }
    else 
#line 115
      {
        q->next[q->tail] = id;
        q->tail = id;
      }
    }
  return TRUE;
}

# 49 "/Users/jingyuancheng/tinyos/moteiv/tos/lib/sched/TaskBasic.nc"
static result_t /*MSP430ArbiterTimerAC.ArbiterC.ArbiterP*/FcfsArbiterP$0$GrantTask$postTask(void ){
#line 49
  unsigned char __nesc_result;
#line 49

#line 49
  __nesc_result = SchedulerBasicP$TaskBasic$postTask(0U);
#line 49

#line 49
  return __nesc_result;
#line 49
}
#line 49
# 59 "/Users/jingyuancheng/tinyos/moteiv/tos/lib/timer/TransformCounterC.nc"
static /*Counter32khzC.Transform*/TransformCounterC$1$to_size_type /*Counter32khzC.Transform*/TransformCounterC$1$Counter$get(void )
{
  /*Counter32khzC.Transform*/TransformCounterC$1$to_size_type rv = 0;

#line 62
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
    {
      /*Counter32khzC.Transform*/TransformCounterC$1$upper_count_type high = /*Counter32khzC.Transform*/TransformCounterC$1$m_upper;
      /*Counter32khzC.Transform*/TransformCounterC$1$from_size_type low = /*Counter32khzC.Transform*/TransformCounterC$1$CounterFrom$get();

#line 66
      if (/*Counter32khzC.Transform*/TransformCounterC$1$CounterFrom$isOverflowPending()) 
        {






          high++;
          low = /*Counter32khzC.Transform*/TransformCounterC$1$CounterFrom$get();
        }
      {
        /*Counter32khzC.Transform*/TransformCounterC$1$to_size_type high_to = high;
        /*Counter32khzC.Transform*/TransformCounterC$1$to_size_type low_to = low >> /*Counter32khzC.Transform*/TransformCounterC$1$LOW_SHIFT_RIGHT;

#line 80
        rv = (high_to << /*Counter32khzC.Transform*/TransformCounterC$1$HIGH_SHIFT_LEFT) | low_to;
      }
    }
#line 82
    __nesc_atomic_end(__nesc_atomic); }
  return rv;
}

# 50 "/Users/jingyuancheng/tinyos/moteiv/tos/platform/msp430/timer/MSP430TimerM.nc"
static uint16_t /*MSP430TimerC.MSP430TimerB*/MSP430TimerM$1$Timer$get(void )
{




  if (1) {
      { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 57
        {
          uint16_t t0;
          uint16_t t1 = * (volatile uint16_t *)400U;

#line 60
          do {
#line 60
              t0 = t1;
#line 60
              t1 = * (volatile uint16_t *)400U;
            }
          while (
#line 60
          t0 != t1);
          {
            unsigned int __nesc_temp = 
#line 61
            t0;

            {
#line 61
              __nesc_atomic_end(__nesc_atomic); 
#line 61
              return __nesc_temp;
            }
          }
        }
#line 64
        __nesc_atomic_end(__nesc_atomic); }
    }
  else 
#line 64
    {
      return * (volatile uint16_t *)400U;
    }
}

# 208 "/Users/jingyuancheng/tinyos/moteiv/tos/platform/tmote/HPLCC2420InterruptM.nc"
static result_t HPLCC2420InterruptM$SFD$disable(void )
#line 208
{
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 209
    {
      HPLCC2420InterruptM$SFDControl$disableEvents();
      HPLCC2420InterruptM$SFDControl$clearPendingInterrupt();
      TOSH_SEL_CC_SFD_IOFUNC();
    }
#line 213
    __nesc_atomic_end(__nesc_atomic); }
  return SUCCESS;
}

# 72 "/Users/jingyuancheng/tinyos/moteiv/tos/platform/msp430/timer/MSP430AlarmC.nc"
static void /*CC2420RadioC.BackoffAlarm32khzC.MSP430Alarm*/MSP430AlarmC$1$Alarm$startAt(uint16_t t0, uint16_t dt)
{
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
    {
      uint16_t now = /*CC2420RadioC.BackoffAlarm32khzC.MSP430Alarm*/MSP430AlarmC$1$MSP430Timer$get();
      uint16_t elapsed = now - t0;

#line 78
      if (elapsed >= dt) 
        {
          /*CC2420RadioC.BackoffAlarm32khzC.MSP430Alarm*/MSP430AlarmC$1$MSP430Compare$setEventFromNow(2);
        }
      else 
        {
          uint16_t remaining = dt - elapsed;

#line 85
          if (remaining <= 2) {
            /*CC2420RadioC.BackoffAlarm32khzC.MSP430Alarm*/MSP430AlarmC$1$MSP430Compare$setEventFromNow(2);
            }
          else {
#line 88
            /*CC2420RadioC.BackoffAlarm32khzC.MSP430Alarm*/MSP430AlarmC$1$MSP430Compare$setEvent(now + remaining);
            }
        }
#line 90
      /*CC2420RadioC.BackoffAlarm32khzC.MSP430Alarm*/MSP430AlarmC$1$MSP430TimerControl$clearPendingInterrupt();
      /*CC2420RadioC.BackoffAlarm32khzC.MSP430Alarm*/MSP430AlarmC$1$MSP430TimerControl$enableEvents();
    }
#line 92
    __nesc_atomic_end(__nesc_atomic); }
}

# 49 "/Users/jingyuancheng/tinyos/moteiv/tos/lib/sched/TaskBasic.nc"
static result_t CC2420RadioM$sendFailedTask$postTask(void ){
#line 49
  unsigned char __nesc_result;
#line 49

#line 49
  __nesc_result = SchedulerBasicP$TaskBasic$postTask(CC2420RadioM$sendFailedTask);
#line 49

#line 49
  return __nesc_result;
#line 49
}
#line 49
static result_t CC2420RadioM$PacketSent$postTask(void ){
#line 49
  unsigned char __nesc_result;
#line 49

#line 49
  __nesc_result = SchedulerBasicP$TaskBasic$postTask(CC2420RadioM$PacketSent);
#line 49

#line 49
  return __nesc_result;
#line 49
}
#line 49
# 80 "/Users/jingyuancheng/tinyos/moteiv/tos/lib/resource/FcfsArbiterP.nc"
static void /*MSP430ArbiterUSART0C.ArbiterC.ArbiterP*/FcfsArbiterP$1$Resource$request(uint8_t id)
#line 80
{
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 81
    {
      if (/*MSP430ArbiterUSART0C.ArbiterC.ArbiterP*/FcfsArbiterP$1$m_granted == RESOURCE_NONE) {
          /*MSP430ArbiterUSART0C.ArbiterC.ArbiterP*/FcfsArbiterP$1$m_granted = id;
        }
      else {
          rqueue_push(/*MSP430ArbiterUSART0C.ArbiterC.ArbiterP*/FcfsArbiterP$1$queue(), id);
          {
#line 87
            __nesc_atomic_end(__nesc_atomic); 
#line 87
            return;
          }
        }
    }
#line 90
    __nesc_atomic_end(__nesc_atomic); }
#line 90
  /*MSP430ArbiterUSART0C.ArbiterC.ArbiterP*/FcfsArbiterP$1$GrantTask$postTask();
}

# 44 "/Users/jingyuancheng/tinyos/moteiv/tos/platform/msp430/RandomMLCG.nc"
static uint16_t RandomMLCG$Random$rand(void )
#line 44
{
  uint32_t mlcg;
#line 45
  uint32_t p;
#line 45
  uint32_t q;
  uint64_t tmpseed;

#line 47
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
    {
      tmpseed = (uint64_t )33614U * (uint64_t )RandomMLCG$seed;
      q = tmpseed;
      q = q >> 1;
      p = tmpseed >> 32;
      mlcg = p + q;
      if (mlcg & 0x80000000) {
          mlcg = mlcg & 0x7FFFFFFF;
          mlcg++;
        }
      RandomMLCG$seed = mlcg;
    }
#line 59
    __nesc_atomic_end(__nesc_atomic); }
  return (uint16_t )mlcg;
}

# 69 "/Users/jingyuancheng/tinyos/moteiv/tos/lib/util/circularQueue.h"
static bool cqueue_isEmpty(CircularQueue_t *cq)
{
  return cq->front == cq->size ? TRUE : FALSE;
}

# 70 "/Users/jingyuancheng/tinyos/moteiv/tos/lib/timer/TransformAlarmC.nc"
static void /*HalTimerMilliC.AlarmMilliC.Transform*/TransformAlarmC$0$set_alarm(void )
{
  /*HalTimerMilliC.AlarmMilliC.Transform*/TransformAlarmC$0$to_size_type now = /*HalTimerMilliC.AlarmMilliC.Transform*/TransformAlarmC$0$Counter$get();
  /*HalTimerMilliC.AlarmMilliC.Transform*/TransformAlarmC$0$from_size_type now_from = now << 5;
  /*HalTimerMilliC.AlarmMilliC.Transform*/TransformAlarmC$0$to_size_type elapsed = now - /*HalTimerMilliC.AlarmMilliC.Transform*/TransformAlarmC$0$m_t0;

#line 75
  if (elapsed >= /*HalTimerMilliC.AlarmMilliC.Transform*/TransformAlarmC$0$m_dt) 
    {
      /*HalTimerMilliC.AlarmMilliC.Transform*/TransformAlarmC$0$m_t0 += /*HalTimerMilliC.AlarmMilliC.Transform*/TransformAlarmC$0$m_dt;
      /*HalTimerMilliC.AlarmMilliC.Transform*/TransformAlarmC$0$m_dt = 0;
      /*HalTimerMilliC.AlarmMilliC.Transform*/TransformAlarmC$0$AlarmFrom$startAt(now_from, 0);
    }
  else 
    {
      /*HalTimerMilliC.AlarmMilliC.Transform*/TransformAlarmC$0$to_size_type remaining = /*HalTimerMilliC.AlarmMilliC.Transform*/TransformAlarmC$0$m_dt - elapsed;
      /*HalTimerMilliC.AlarmMilliC.Transform*/TransformAlarmC$0$from_size_type remaining_from = remaining;

#line 85
      if (remaining > /*HalTimerMilliC.AlarmMilliC.Transform*/TransformAlarmC$0$MAX_DELAY) 
        {
          /*HalTimerMilliC.AlarmMilliC.Transform*/TransformAlarmC$0$m_t0 = now + /*HalTimerMilliC.AlarmMilliC.Transform*/TransformAlarmC$0$MAX_DELAY;
          /*HalTimerMilliC.AlarmMilliC.Transform*/TransformAlarmC$0$m_dt = remaining - /*HalTimerMilliC.AlarmMilliC.Transform*/TransformAlarmC$0$MAX_DELAY;
          /*HalTimerMilliC.AlarmMilliC.Transform*/TransformAlarmC$0$AlarmFrom$startAt(now_from, (/*HalTimerMilliC.AlarmMilliC.Transform*/TransformAlarmC$0$from_size_type )/*HalTimerMilliC.AlarmMilliC.Transform*/TransformAlarmC$0$MAX_DELAY << 5);
        }
      else 
        {
          /*HalTimerMilliC.AlarmMilliC.Transform*/TransformAlarmC$0$m_t0 += /*HalTimerMilliC.AlarmMilliC.Transform*/TransformAlarmC$0$m_dt;
          /*HalTimerMilliC.AlarmMilliC.Transform*/TransformAlarmC$0$m_dt = 0;
          /*HalTimerMilliC.AlarmMilliC.Transform*/TransformAlarmC$0$AlarmFrom$startAt(now_from, remaining_from << 5);
        }
    }
}

# 59 "/Users/jingyuancheng/tinyos/moteiv/tos/lib/timer/TransformCounterC.nc"
static /*CounterMilliC.Transform*/TransformCounterC$0$to_size_type /*CounterMilliC.Transform*/TransformCounterC$0$Counter$get(void )
{
  /*CounterMilliC.Transform*/TransformCounterC$0$to_size_type rv = 0;

#line 62
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
    {
      /*CounterMilliC.Transform*/TransformCounterC$0$upper_count_type high = /*CounterMilliC.Transform*/TransformCounterC$0$m_upper;
      /*CounterMilliC.Transform*/TransformCounterC$0$from_size_type low = /*CounterMilliC.Transform*/TransformCounterC$0$CounterFrom$get();

#line 66
      if (/*CounterMilliC.Transform*/TransformCounterC$0$CounterFrom$isOverflowPending()) 
        {






          high++;
          low = /*CounterMilliC.Transform*/TransformCounterC$0$CounterFrom$get();
        }
      {
        /*CounterMilliC.Transform*/TransformCounterC$0$to_size_type high_to = high;
        /*CounterMilliC.Transform*/TransformCounterC$0$to_size_type low_to = low >> /*CounterMilliC.Transform*/TransformCounterC$0$LOW_SHIFT_RIGHT;

#line 80
        rv = (high_to << /*CounterMilliC.Transform*/TransformCounterC$0$HIGH_SHIFT_LEFT) | low_to;
      }
    }
#line 82
    __nesc_atomic_end(__nesc_atomic); }
  return rv;
}

# 72 "/Users/jingyuancheng/tinyos/moteiv/tos/platform/msp430/timer/MSP430AlarmC.nc"
static void /*HalTimerMilliC.AlarmMilliC.MSP430Alarm*/MSP430AlarmC$0$Alarm$startAt(uint16_t t0, uint16_t dt)
{
  /* atomic removed: atomic calls only */
  {
    uint16_t now = /*HalTimerMilliC.AlarmMilliC.MSP430Alarm*/MSP430AlarmC$0$MSP430Timer$get();
    uint16_t elapsed = now - t0;

#line 78
    if (elapsed >= dt) 
      {
        /*HalTimerMilliC.AlarmMilliC.MSP430Alarm*/MSP430AlarmC$0$MSP430Compare$setEventFromNow(2);
      }
    else 
      {
        uint16_t remaining = dt - elapsed;

#line 85
        if (remaining <= 2) {
          /*HalTimerMilliC.AlarmMilliC.MSP430Alarm*/MSP430AlarmC$0$MSP430Compare$setEventFromNow(2);
          }
        else {
#line 88
          /*HalTimerMilliC.AlarmMilliC.MSP430Alarm*/MSP430AlarmC$0$MSP430Compare$setEvent(now + remaining);
          }
      }
#line 90
    /*HalTimerMilliC.AlarmMilliC.MSP430Alarm*/MSP430AlarmC$0$MSP430TimerControl$clearPendingInterrupt();
    /*HalTimerMilliC.AlarmMilliC.MSP430Alarm*/MSP430AlarmC$0$MSP430TimerControl$enableEvents();
  }
}

# 70 "/Users/jingyuancheng/tinyos/moteiv/tos/lib/timer/TransformAlarmC.nc"
static void /*CC2420SyncAlwaysOnC.AlarmC.Transform*/TransformAlarmC$1$set_alarm(void )
{
  /*CC2420SyncAlwaysOnC.AlarmC.Transform*/TransformAlarmC$1$to_size_type now = /*CC2420SyncAlwaysOnC.AlarmC.Transform*/TransformAlarmC$1$Counter$get();
  /*CC2420SyncAlwaysOnC.AlarmC.Transform*/TransformAlarmC$1$from_size_type now_from = now << 0;
  /*CC2420SyncAlwaysOnC.AlarmC.Transform*/TransformAlarmC$1$to_size_type elapsed = now - /*CC2420SyncAlwaysOnC.AlarmC.Transform*/TransformAlarmC$1$m_t0;

#line 75
  if (elapsed >= /*CC2420SyncAlwaysOnC.AlarmC.Transform*/TransformAlarmC$1$m_dt) 
    {
      /*CC2420SyncAlwaysOnC.AlarmC.Transform*/TransformAlarmC$1$m_t0 += /*CC2420SyncAlwaysOnC.AlarmC.Transform*/TransformAlarmC$1$m_dt;
      /*CC2420SyncAlwaysOnC.AlarmC.Transform*/TransformAlarmC$1$m_dt = 0;
      /*CC2420SyncAlwaysOnC.AlarmC.Transform*/TransformAlarmC$1$AlarmFrom$startAt(now_from, 0);
    }
  else 
    {
      /*CC2420SyncAlwaysOnC.AlarmC.Transform*/TransformAlarmC$1$to_size_type remaining = /*CC2420SyncAlwaysOnC.AlarmC.Transform*/TransformAlarmC$1$m_dt - elapsed;
      /*CC2420SyncAlwaysOnC.AlarmC.Transform*/TransformAlarmC$1$from_size_type remaining_from = remaining;

#line 85
      if (remaining > /*CC2420SyncAlwaysOnC.AlarmC.Transform*/TransformAlarmC$1$MAX_DELAY) 
        {
          /*CC2420SyncAlwaysOnC.AlarmC.Transform*/TransformAlarmC$1$m_t0 = now + /*CC2420SyncAlwaysOnC.AlarmC.Transform*/TransformAlarmC$1$MAX_DELAY;
          /*CC2420SyncAlwaysOnC.AlarmC.Transform*/TransformAlarmC$1$m_dt = remaining - /*CC2420SyncAlwaysOnC.AlarmC.Transform*/TransformAlarmC$1$MAX_DELAY;
          /*CC2420SyncAlwaysOnC.AlarmC.Transform*/TransformAlarmC$1$AlarmFrom$startAt(now_from, (/*CC2420SyncAlwaysOnC.AlarmC.Transform*/TransformAlarmC$1$from_size_type )/*CC2420SyncAlwaysOnC.AlarmC.Transform*/TransformAlarmC$1$MAX_DELAY << 0);
        }
      else 
        {
          /*CC2420SyncAlwaysOnC.AlarmC.Transform*/TransformAlarmC$1$m_t0 += /*CC2420SyncAlwaysOnC.AlarmC.Transform*/TransformAlarmC$1$m_dt;
          /*CC2420SyncAlwaysOnC.AlarmC.Transform*/TransformAlarmC$1$m_dt = 0;
          /*CC2420SyncAlwaysOnC.AlarmC.Transform*/TransformAlarmC$1$AlarmFrom$startAt(now_from, remaining_from << 0);
        }
    }
}

# 72 "/Users/jingyuancheng/tinyos/moteiv/tos/platform/msp430/timer/MSP430AlarmC.nc"
static void /*CC2420SyncAlwaysOnC.AlarmC.MSP430Alarm*/MSP430AlarmC$2$Alarm$startAt(uint16_t t0, uint16_t dt)
{
  /* atomic removed: atomic calls only */
  {
    uint16_t now = /*CC2420SyncAlwaysOnC.AlarmC.MSP430Alarm*/MSP430AlarmC$2$MSP430Timer$get();
    uint16_t elapsed = now - t0;

#line 78
    if (elapsed >= dt) 
      {
        /*CC2420SyncAlwaysOnC.AlarmC.MSP430Alarm*/MSP430AlarmC$2$MSP430Compare$setEventFromNow(2);
      }
    else 
      {
        uint16_t remaining = dt - elapsed;

#line 85
        if (remaining <= 2) {
          /*CC2420SyncAlwaysOnC.AlarmC.MSP430Alarm*/MSP430AlarmC$2$MSP430Compare$setEventFromNow(2);
          }
        else {
#line 88
          /*CC2420SyncAlwaysOnC.AlarmC.MSP430Alarm*/MSP430AlarmC$2$MSP430Compare$setEvent(now + remaining);
          }
      }
#line 90
    /*CC2420SyncAlwaysOnC.AlarmC.MSP430Alarm*/MSP430AlarmC$2$MSP430TimerControl$clearPendingInterrupt();
    /*CC2420SyncAlwaysOnC.AlarmC.MSP430Alarm*/MSP430AlarmC$2$MSP430TimerControl$enableEvents();
  }
}

# 14 "/Users/jingyuancheng/tinyos/moteiv/tos/platform/msp430/timer/MSP430TimerCommonM.nc"
 __attribute((wakeup)) __attribute((interrupt(24))) void sig_TIMERB1_VECTOR(void )
#line 14
{
#line 14
  MSP430TimerCommonM$VectorTimerB1$fired();
}

# 43 "/Users/jingyuancheng/tinyos/moteiv/tos/lib/sched/MainP.nc"
  int main(void )
#line 43
{
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 44
    {
      MainP$Scheduler$init();
      MainP$init();
      while (MainP$Scheduler$runNextTask(FALSE)) ;
    }
#line 48
    __nesc_atomic_end(__nesc_atomic); }


  __nesc_enable_interrupt();

  MainP$m_starting = 255;
  MainP$startDone(255);




  for (; ; ) {
      MainP$Scheduler$runNextTask(TRUE);
    }
}

# 88 "/Users/jingyuancheng/tinyos/moteiv/tos/platform/tmote/hardware.h"
static void TOSH_FLASH_M25P_DP_bit(bool set)
#line 88
{
  if (set) {
    TOSH_SET_SIMO0_PIN();
    }
  else {
#line 92
    TOSH_CLR_SIMO0_PIN();
    }
#line 93
  TOSH_SET_UCLK0_PIN();
  TOSH_CLR_UCLK0_PIN();
}

# 151 "/Users/jingyuancheng/tinyos/moteiv/tos/platform/msp430/timer/MSP430ClockM.nc"
static void MSP430ClockM$set_dco_calib(int calib)
{
  BCSCTL1 = (BCSCTL1 & ~0x07) | ((calib >> 8) & 0x07);
  DCOCTL = calib & 0xff;
}

# 26 "/Users/jingyuancheng/tinyos/moteiv/tos/lib/util/reservedQueue.h"
static void rqueue_init(ReservedQueue_t *q, uint8_t count)
#line 26
{
  uint8_t *next = q->next;
  const uint8_t *nextEnd = next + count;

  q->head = RQUEUE_NONE;
  q->tail = RQUEUE_NONE;

  while (next != nextEnd) 
    * next++ = RQUEUE_NONE;
}

# 284 "/Users/jingyuancheng/tinyos/moteiv/tos/platform/tmote/FramerP.nc"
static void FramerP$HDLCInitialize(void )
#line 284
{
  int i;

#line 286
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 286
    {
      for (i = 0; i < FramerP$HDLC_QUEUESIZE; i++) {
          FramerP$gMsgRcvTbl[i].pMsg = &FramerP$gMsgRcvBuf[i];
          FramerP$gMsgRcvTbl[i].Length = 0;
          FramerP$gMsgRcvTbl[i].Token = 0;
        }
      FramerP$gTxState = FramerP$TXSTATE_IDLE;
      FramerP$gTxByteCnt = 0;
      FramerP$gTxLength = 0;
      FramerP$gTxRunningCRC = 0;
      FramerP$gpTxMsg = NULL;

      FramerP$gRxState = FramerP$RXSTATE_NOSYNC;
      FramerP$gRxHeadIndex = 0;
      FramerP$gRxTailIndex = 0;
      FramerP$gRxByteCnt = 0;
      FramerP$gRxRunningCRC = 0;
      FramerP$gpRxBuf = (uint8_t *)FramerP$gMsgRcvTbl[FramerP$gRxHeadIndex].pMsg;
    }
#line 304
    __nesc_atomic_end(__nesc_atomic); }
}

# 155 "/Users/jingyuancheng/tinyos/moteiv/tos/lib/sched/SchedulerBasicP.nc"
static bool SchedulerBasicP$Scheduler$runNextTask(bool sleep)
#line 155
{
  uint8_t nextTask;

#line 157
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 157
    {
#line 173
      nextTask = SchedulerBasicP$popTask();
      if (nextTask == SchedulerBasicP$NONE) {
          if (sleep) {



              __nesc_atomic_sleep();
            }
          {
            unsigned char __nesc_temp = 
#line 181
            FALSE;

            {
#line 181
              __nesc_atomic_end(__nesc_atomic); 
#line 181
              return __nesc_temp;
            }
          }
        }
    }
#line 185
    __nesc_atomic_end(__nesc_atomic); }
#line 184
  SchedulerBasicP$TaskBasic$runTask(nextTask);
  return TRUE;
}

# 574 "/Users/jingyuancheng/tinyos/moteiv/tos/lib/CC2420Radio/CC2420RadioM.nc"
static void CC2420RadioM$tryToSend(uint8_t rh)
#line 574
{
  uint8_t currentstate;

#line 576
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 576
    currentstate = CC2420RadioM$stateRadio;
#line 576
    __nesc_atomic_end(__nesc_atomic); }


  if (currentstate == CC2420RadioM$PRE_TX_STATE) {

      if (CC2420RadioM$bShutdownRequest) {
          CC2420RadioM$sendFailedAsync();
          return;
        }



      if (!TOSH_READ_CC_FIFO_PIN() && !TOSH_READ_CC_FIFOP_PIN()) {
          CC2420RadioM$flushRXFIFO(rh);
        }

      if (TOSH_READ_RADIO_CCA_PIN()) {
          { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 593
            CC2420RadioM$stateRadio = CC2420RadioM$TX_STATE;
#line 593
            __nesc_atomic_end(__nesc_atomic); }
          CC2420RadioM$sendPacket(rh);
        }
      else {



          if (CC2420RadioM$countRetry-- <= 0) {
              CC2420RadioM$flushRXFIFO(rh);
              CC2420RadioM$countRetry = 8;
              CC2420RadioM$CmdTransmit$deferRequest();
              return;
            }
          if (!CC2420RadioM$setBackoffTimer(CC2420RadioM$MacBackoff$congestionBackoff(CC2420RadioM$txbufptr) * 2)) {
              CC2420RadioM$sendFailedAsync();
            }
        }
    }
}

#line 145
static void CC2420RadioM$sendFailedSync(void )
#line 145
{
  cc2420_error_t _error = CC2420_E_UNKNOWN;

#line 147
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 147
    CC2420RadioM$stateRadio = CC2420RadioM$IDLE_STATE;
#line 147
    __nesc_atomic_end(__nesc_atomic); }
  CC2420RadioM$txbufptr->length = CC2420RadioM$txbufptr->length - MSG_HEADER_SIZE - MSG_FOOTER_SIZE;
  if (CC2420RadioM$bShutdownRequest) {
    _error = CC2420_E_SHUTDOWN;
    }
#line 151
  CC2420RadioM$Send$sendDone(CC2420RadioM$txbufptr, _error);
}

# 115 "/Users/jingyuancheng/tinyos/moteiv/tos/lib/sp/cc2420/CC2420AlwaysOnM.nc"
static result_t CC2420AlwaysOnM$LowerSend$sendDone(TOS_MsgPtr msg, cc2420_error_t success)
#line 115
{
  sp_message_t *_stack = CC2420AlwaysOnM$spmsg;
  sp_error_t _error = SP_SUCCESS;

#line 118
  CC2420AlwaysOnM$spmsg = NULL;

  if (_stack->flags & SP_FLAG_C_RELIABLE && 
  _stack->addr != TOS_BCAST_ADDR) {
      if (msg->ack) {
          _stack->flags |= SP_FLAG_F_RELIABLE;
        }
      else {
          _error = SP_E_RELIABLE;
        }
    }


  if (_stack->flags & SP_FLAG_C_TIMESTAMP) {
      _stack->msg->length = _stack->length;
    }

  if (success == CC2420_E_SHUTDOWN) {
      _error = SP_E_SHUTDOWN;
    }

  if (_error == SP_SUCCESS && success != CC2420_SUCCESS) {
    _error = SP_E_UNKNOWN;
    }
  if (CC2420AlwaysOnM$m_backoffs > 5) {
    _stack->flags |= SP_FLAG_F_CONGESTION;
    }
  CC2420AlwaysOnM$m_flags = CC2420AlwaysOnM$FLAG_SENDDONE;
  CC2420AlwaysOnM$SPSend$sendDone(_stack, _stack->flags, _error);
  CC2420AlwaysOnM$m_flags &= ~CC2420AlwaysOnM$FLAG_SENDDONE;

  return SUCCESS;
}

# 244 "/Users/jingyuancheng/tinyos/moteiv/tos/lib/sp/SPM.nc"
static void SPM$processSendComplete(sp_message_t *msg, sp_message_flags_t flags, sp_error_t success)
#line 244
{
  sp_message_t *_stack;
  TOS_Msg *_tosstack;


  if (success == SP_E_SHUTDOWN) {
    SPM$tryNextSend();
    }
  else {
    if (
#line 252
    success != SP_SUCCESS && 
    SPM$m_currentmsg->flags & SP_FLAG_C_RELIABLE && 
    ++ msg->retries < 3) {

        SPM$m_currentmsg->flags &= ~SP_FLAG_F_RELIABLE;
        SPM$tryNextSend();
      }
    else 
      {

        _stack = SPM$m_currentmsg;
        _tosstack = SPM$m_currentmsg->msg;
        _stack->flags &= ~SP_FLAG_C_BUSY;


        SPM$m_currentmsg->quantity--;


        if (SPM$m_currentmsg->quantity > 0 && success == SP_SUCCESS) {
            SPM$m_currentmsg->msg = NULL;
            SPM$m_currentmsg->flags |= SP_FLAG_C_FUTURES;

            SPM$SPSendNext$request(SPM$m_currentmsg->id, SPM$m_currentmsg, 
            _tosstack, 
            SPM$m_currentmsg->quantity);

            SPM$m_currentmsg->flags &= ~SP_FLAG_C_FUTURES;
            SPM$m_currentmsg->retries = 0;

            if (SPM$m_currentmsg->msg == NULL) {
                SPM$m_currentmsg = NULL;
                SPM$m_sending = FALSE;
                SPM$Pool$remove(_stack);


                SPM$SPSend$sendDone(_stack->id, _stack, 
                _stack->flags, 
                SP_E_BUF_UNDERRUN);
              }
            else 


              {
                SPM$tryNextSend();
              }
          }
        else 
          {
            SPM$m_currentmsg = NULL;
            SPM$m_sending = FALSE;
            SPM$Pool$remove(_stack);
            SPM$SPSend$sendDone(_stack->id, _stack, 
            _stack->flags, 
            success);
          }
      }
    }
}

#line 218
static void SPM$tryNextSend(void )
#line 218
{





  if ((
#line 223
  SPM$isOkToSend(SPM$m_currentmsg->addr) || 
  SPM$neighborPopulation() == 0) && 
  SPM$SPLinkStats$getState() == SP_RADIO_ON) {








      if (
#line 227
      SPM$LowerSend$sendAdv(SPM$m_currentmsg, 
      SPM$m_currentmsg->msg, 
      SPM$m_currentmsg->dev, 
      SPM$m_currentmsg->addr, 
      SPM$m_currentmsg->length, 
      SPM$m_currentmsg->flags, 
      SPM$m_currentmsg->quantity)
       == SUCCESS) {
          SPM$m_currentmsg->flags |= SP_FLAG_C_BUSY;
          return;
        }
    }

  SPM$m_currentmsg = NULL;
  SPM$m_sending = FALSE;
}

#line 48
static bool SPM$isOkToSend(uint16_t addr)
#line 48
{
  uint8_t j;

#line 50
  for (j = SPM$SPNeighbor$first(); SPM$SPNeighbor$valid(j); j = SPM$SPNeighbor$next(j)) {
      sp_neighbor_t *n = SPM$SPNeighbor$get(j);


      if ((
#line 53
      n->addr == addr || 
      n->addr == TOS_BCAST_ADDR) && 
      n->flags & SP_FLAG_LINK_ACTIVE) {
          return TRUE;
        }
    }
  return FALSE;
}

# 105 "/Users/jingyuancheng/tinyos/moteiv/tos/lib/util/pool/ObjectPoolC.nc"
static uint8_t /*SPC.NeighborTable*/ObjectPoolC$1$Pool$next(uint8_t n)
#line 105
{
  if (n < 10) {
      for (n++; n < 10; n++) {
          if (/*SPC.NeighborTable*/ObjectPoolC$1$m_pool[n] != NULL) {
            break;
            }
        }
    }
#line 112
  return n;
}

#line 82
static uint8_t /*SPC.NeighborTable*/ObjectPoolC$1$Pool$populated(void )
#line 82
{
  /*SPC.NeighborTable*/ObjectPoolC$1$object_type **p;
  /*SPC.NeighborTable*/ObjectPoolC$1$object_type **pend = /*SPC.NeighborTable*/ObjectPoolC$1$m_pool + 10;
  uint8_t num = 0;

#line 86
  for (p = /*SPC.NeighborTable*/ObjectPoolC$1$m_pool + 0; p != pend; p++) {
      if (*p != NULL) {
        num++;
        }
    }
#line 90
  return num;
}

# 76 "/Users/jingyuancheng/tinyos/moteiv/tos/lib/sp/cc2420/CC2420AlwaysOnM.nc"
static result_t CC2420AlwaysOnM$SPSend$sendAdv(sp_message_t *_spmsg, TOS_Msg *_tosmsg, sp_device_t _dev, sp_address_t _addr, uint8_t _length, sp_message_flags_t _flags, uint8_t _quantity)
#line 76
{
  if (CC2420AlwaysOnM$spmsg != NULL) {
      return FAIL;
    }
  if (_spmsg->flags & SP_FLAG_C_TIMESTAMP) {

      if (CC2420AlwaysOnM$TimeStamping$addStamp(_spmsg->msg, _length) != SUCCESS) {
          return FAIL;
        }
      else {
          _spmsg->msg->length = _length + 4;
        }
    }

  if (CC2420AlwaysOnM$LowerSend$send(_spmsg->msg) == SUCCESS) {
      if (_spmsg->flags & SP_FLAG_C_RELIABLE && 
      _spmsg->addr != TOS_BCAST_ADDR) {

          CC2420AlwaysOnM$MacControl$requestAck(_spmsg->msg);
        }

      CC2420AlwaysOnM$m_backoffs = 0;
      CC2420AlwaysOnM$spmsg = _spmsg;
      if (CC2420AlwaysOnM$m_flags & CC2420AlwaysOnM$FLAG_SENDDONE) {
          CC2420AlwaysOnM$m_flags |= CC2420AlwaysOnM$FLAG_MULTIMSG;
        }
      return SUCCESS;
    }
  return FAIL;
}

# 51 "/Users/jingyuancheng/tinyos/moteiv/tos/lib/util/pool/ObjectPoolC.nc"
static result_t /*SPC.MessagePool*/ObjectPoolC$0$Pool$remove(/*SPC.MessagePool*/ObjectPoolC$0$object_type *obj)
#line 51
{
  /*SPC.MessagePool*/ObjectPoolC$0$object_type **p;
  /*SPC.MessagePool*/ObjectPoolC$0$object_type **pend = /*SPC.MessagePool*/ObjectPoolC$0$m_pool + 10;

#line 54
  for (p = /*SPC.MessagePool*/ObjectPoolC$0$m_pool + 0; p != pend; p++) {
      if (*p == obj) {
          *p = NULL;
          /*SPC.MessagePool*/ObjectPoolC$0$PoolEvents$removed(obj);
          return SUCCESS;
        }
    }
  return FAIL;
}

# 65 "/Users/jingyuancheng/tinyos/moteiv/tos/lib/sp/SPAdaptorGenericCommM.nc"
static void SPAdaptorGenericCommM$SPSend$sendDone(uint8_t id, sp_message_t *_msg, sp_message_flags_t flags, sp_error_t _success)
#line 65
{






  if (SPAdaptorGenericCommM$contains(_msg)) {
      TOS_MsgPtr _stack = _msg->msg;

#line 74
      _msg->msg = NULL;
      SPAdaptorGenericCommM$SendMsg$sendDone(_stack->type, _stack, _success == SP_SUCCESS);
    }
}

# 66 "/Users/jingyuancheng/tinyos/moteiv/tos/lib/sp/SPM.nc"
static void SPM$nextSend(void )
#line 66
{
  bool urgent = FALSE;
  uint32_t time = SPM$LocalTime$get();
  uint32_t min = (uint32_t )0xFFFFFFFF;

  if (SPM$m_sending && SPM$m_currentmsg != NULL) {
    return;
    }
  SPM$m_sending = FALSE;
  SPM$m_currentmsg = NULL;


  if (SPM$SPLinkStats$getState() == SP_RADIO_ON) {

      uint8_t i;

#line 81
      for (i = SPM$Pool$first(); SPM$Pool$valid(i); i = SPM$Pool$next(i)) {

          sp_message_t *temp = SPM$Pool$get(i);




          if (SPM$isOkToSend(temp->addr) || SPM$neighborPopulation() == 0) {

              if (!urgent) {

                  if (temp->flags & SP_FLAG_C_URGENT) {
                      urgent = TRUE;
                      min = time - temp->time;
                      SPM$m_currentmsg = temp;
                    }
                  else {
#line 97
                    if (time - temp->time <= min) {
                        min = time - temp->time;
                        SPM$m_currentmsg = temp;
                      }
                    }
                }
              else {
                  if (temp->flags & SP_FLAG_C_URGENT && 
                  time - temp->time <= min) {
                      min = time - temp->time;
                      SPM$m_currentmsg = temp;
                    }
                }
            }
        }
    }



  if (SPM$m_currentmsg != NULL) {

      SPM$m_currentmsg->msg->type = SPM$m_currentmsg->id;







      if (
#line 120
      SPM$LowerSend$sendAdv(SPM$m_currentmsg, 
      SPM$m_currentmsg->msg, 
      SPM$m_currentmsg->dev, 
      SPM$m_currentmsg->addr, 
      SPM$m_currentmsg->length, 
      SPM$m_currentmsg->flags, 
      SPM$m_currentmsg->quantity) == SUCCESS) {
          SPM$m_sending = TRUE;
          SPM$m_currentmsg->flags |= SP_FLAG_C_BUSY;
        }
      else {

          SPM$m_currentmsg->retries++;


          if (SPM$m_currentmsg->retries >= 3) {
              SPM$processSendComplete(SPM$m_currentmsg, SPM$m_currentmsg->flags, SP_E_UNKNOWN);
            }
          else {
              SPM$m_currentmsg = NULL;
              SPM$m_sending = FALSE;
            }
        }
    }
}

# 105 "/Users/jingyuancheng/tinyos/moteiv/tos/lib/util/pool/ObjectPoolC.nc"
static uint8_t /*SPC.MessagePool*/ObjectPoolC$0$Pool$next(uint8_t n)
#line 105
{
  if (n < 10) {
      for (n++; n < 10; n++) {
          if (/*SPC.MessagePool*/ObjectPoolC$0$m_pool[n] != NULL) {
            break;
            }
        }
    }
#line 112
  return n;
}

# 163 "/Users/jingyuancheng/tinyos/moteiv/tos/lib/resource/FcfsArbiterP.nc"
static void /*MSP430ArbiterUSART0C.ArbiterC.ArbiterP*/FcfsArbiterP$1$ResourceCmdAsync$request(uint8_t id, uint8_t rh)
#line 163
{
  bool grant = TRUE;

#line 165
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 165
    {
      if (/*MSP430ArbiterUSART0C.ArbiterC.ArbiterP*/FcfsArbiterP$1$m_granted == RESOURCE_NONE) {
          /*MSP430ArbiterUSART0C.ArbiterC.ArbiterP*/FcfsArbiterP$1$m_granted = rh = id;
          /*MSP430ArbiterUSART0C.ArbiterC.ArbiterP*/FcfsArbiterP$1$ResourceConfigure$configure(id);
        }
      else {
#line 170
        if (rh != /*MSP430ArbiterUSART0C.ArbiterC.ArbiterP*/FcfsArbiterP$1$m_granted) {
            rqueue_push(/*MSP430ArbiterUSART0C.ArbiterC.ArbiterP*/FcfsArbiterP$1$queue(), id);
            grant = FALSE;
          }
        }
    }
#line 175
    __nesc_atomic_end(__nesc_atomic); }
#line 175
  if (grant) {
      /*MSP430ArbiterUSART0C.ArbiterC.ArbiterP*/FcfsArbiterP$1$ResourceCmdAsync$granted(id, rh);
    }
  else {
      /*MSP430ArbiterUSART0C.ArbiterC.ArbiterP*/FcfsArbiterP$1$Arbiter$requested();
    }
}

# 153 "/Users/jingyuancheng/tinyos/moteiv/tos/lib/timer/VirtualizeTimerC.nc"
static void /*HalTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC$0$startTimer(uint8_t num, uint32_t t0, uint32_t dt, bool isoneshot)
{
  /*HalTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC$0$m_timers[num].t0 = t0;
  /*HalTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC$0$m_timers[num].dt = dt;
  /*HalTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC$0$m_flags[num].isoneshot = isoneshot;
  /*HalTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC$0$m_flags[num].isrunning = TRUE;
  /*HalTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC$0$executeTimersNow$postTask();
}

# 49 "/Users/jingyuancheng/tinyos/moteiv/tos/lib/sched/TaskBasic.nc"
static result_t /*HalTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC$0$executeTimersNow$postTask(void ){
#line 49
  unsigned char __nesc_result;
#line 49

#line 49
  __nesc_result = SchedulerBasicP$TaskBasic$postTask(/*HalTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC$0$executeTimersNow);
#line 49

#line 49
  return __nesc_result;
#line 49
}
#line 49
# 385 "/Users/jingyuancheng/tinyos/moteiv/tos/lib/CC2420Radio/CC2420RadioM.nc"
static result_t CC2420RadioM$SplitControl$start(void )
#line 385
{
  uint8_t chkstateRadio;

  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 388
    chkstateRadio = CC2420RadioM$stateRadio;
#line 388
    __nesc_atomic_end(__nesc_atomic); }

  if (chkstateRadio == CC2420RadioM$DISABLED_STATE) {
      { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 391
        {
          CC2420RadioM$stateRadio = CC2420RadioM$WARMUP_STATE;
          CC2420RadioM$countRetry = 0;
          CC2420RadioM$rxbufptr->length = 0;
        }
#line 395
        __nesc_atomic_end(__nesc_atomic); }
      CC2420RadioM$TimerControl$start();
      return CC2420RadioM$CC2420SplitControl$start();
    }
  return FAIL;
}

# 332 "/Users/jingyuancheng/tinyos/moteiv/tos/lib/sp/SPM.nc"
static void SPM$setRxFields(sp_message_t *msg, TOS_Msg *tosmsg, sp_device_t dev, sp_message_flags_t flags, uint8_t quantity, uint8_t id)
#line 332
{

  msg->msg = tosmsg;


  msg->addr = tosmsg->addr;

  msg->dev = dev == SP_I_NOT_SPECIFIED ? SP_I_RADIO : dev;
  msg->dev = msg->addr == TOS_UART_ADDR ? SP_I_UART : msg->dev;

  msg->id = id;

  msg->flags = flags & ~SP_FLAG_F_ALL;

  msg->quantity = quantity;

  msg->retries = 0;
}

# 157 "/Users/jingyuancheng/tinyos/moteiv/tos/platform/tmote/FramerP.nc"
static result_t FramerP$StartTx(void )
#line 157
{
  result_t Result = SUCCESS;
  bool fInitiate = FALSE;

#line 160
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 160
    {
      if (FramerP$gTxState == FramerP$TXSTATE_IDLE) {
          if (FramerP$gFlags & FramerP$FLAGS_TOKENPEND) {
              FramerP$gpTxBuf = (uint8_t *)&FramerP$gTxTokenBuf;
              FramerP$gTxProto = FramerP$PROTO_ACK;
              FramerP$gTxLength = sizeof FramerP$gTxTokenBuf;
              fInitiate = TRUE;
              FramerP$gTxState = FramerP$TXSTATE_PROTO;
            }
          else {
#line 169
            if (FramerP$gFlags & FramerP$FLAGS_DATAPEND) {
                FramerP$gpTxBuf = (uint8_t *)FramerP$gpTxMsg;
                FramerP$gTxProto = FramerP$PROTO_PACKET_NOACK;
                FramerP$gTxLength = FramerP$gpTxMsg->length + (MSG_DATA_SIZE - DATA_LENGTH - 2);
                fInitiate = TRUE;
                FramerP$gTxState = FramerP$TXSTATE_PROTO;
              }
            else {
#line 176
              if (FramerP$gFlags & FramerP$FLAGS_UNKNOWN) {
                  FramerP$gpTxBuf = (uint8_t *)&FramerP$gTxUnknownBuf;
                  FramerP$gTxProto = FramerP$PROTO_UNKNOWN;
                  FramerP$gTxLength = sizeof FramerP$gTxUnknownBuf;
                  fInitiate = TRUE;
                  FramerP$gTxState = FramerP$TXSTATE_PROTO;
                }
              }
            }
        }
    }
#line 186
    __nesc_atomic_end(__nesc_atomic); }
#line 186
  if (fInitiate) {
      { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 187
        {
          FramerP$gTxRunningCRC = 0;
#line 188
          FramerP$gTxByteCnt = 0;
        }
#line 189
        __nesc_atomic_end(__nesc_atomic); }

      FramerP$Timer$stop();
      FramerP$USARTControl$enableUARTTx();


      Result = FramerP$ByteComm$txByte(FramerP$HDLC_FLAG_BYTE);
      if (Result != SUCCESS) {
          { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 197
            FramerP$gTxState = FramerP$TXSTATE_ERROR;
#line 197
            __nesc_atomic_end(__nesc_atomic); }
          FramerP$PacketSent$postTask();
        }
    }

  return Result;
}

# 110 "/Users/jingyuancheng/tinyos/moteiv/tinyos-1.x/tos/system/UARTM.nc"
static result_t UARTM$ByteComm$txByte(uint8_t data)
#line 110
{
  bool oldState;

  {
  }
#line 113
  ;

  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 115
    {
      oldState = UARTM$state;
      UARTM$state = TRUE;
    }
#line 118
    __nesc_atomic_end(__nesc_atomic); }
  if (oldState) {
    return FAIL;
    }
  UARTM$HPLUART$put(data);

  return SUCCESS;
}

# 49 "/Users/jingyuancheng/tinyos/moteiv/tos/lib/sched/TaskBasic.nc"
static result_t FramerP$PacketSent$postTask(void ){
#line 49
  unsigned char __nesc_result;
#line 49

#line 49
  __nesc_result = SchedulerBasicP$TaskBasic$postTask(FramerP$PacketSent);
#line 49

#line 49
  return __nesc_result;
#line 49
}
#line 49
# 362 "/Users/jingyuancheng/tinyos/moteiv/tos/lib/sp/SPM.nc"
static TOS_MsgPtr SPM$UARTReceive$receive(TOS_MsgPtr m)
#line 362
{
  sp_message_t spmsg;

#line 364
  SPM$setRxFields(&spmsg, m, SP_I_UART, SP_FLAG_C_NONE, 1, m->type);
  SPM$SPReceive$receive(m->type, &spmsg, m, SP_SUCCESS);
  return SPM$ReceiveMsg$receive(m->type, m);
}

# 162 "/Users/jingyuancheng/tinyos/moteiv/tos/lib/timer/VirtualizeTimerC.nc"
static void /*HalTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC$0$Timer$startPeriodic(uint8_t num, uint32_t dt)
{
  /*HalTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC$0$startTimer(num, /*HalTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC$0$TimerFrom$getNow(), dt, FALSE);
}

# 332 "/Users/jingyuancheng/tinyos/moteiv/tos/platform/tmote/FramerP.nc"
static result_t FramerP$BareSendMsg$send(TOS_MsgPtr pMsg)
#line 332
{
  result_t Result = SUCCESS;

  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 335
    {
      if (!(FramerP$gFlags & FramerP$FLAGS_DATAPEND)) {
          FramerP$gFlags |= FramerP$FLAGS_DATAPEND;
          FramerP$gpTxMsg = pMsg;
        }
      else 

        {
          Result = FAIL;
        }
    }
#line 345
    __nesc_atomic_end(__nesc_atomic); }

  if (Result == SUCCESS) {
      Result = FramerP$StartTx();
    }

  return Result;
}

# 71 "/Users/jingyuancheng/tinyos/moteiv/tos/lib/timer/VirtualizeTimerC.nc"
static void /*HalTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC$0$executeTimers(uint32_t then)
{
  int32_t min_remaining = (1UL << 31) - 1;
  bool min_remaining_isset = FALSE;
  int num;

  for (num = 0; num < /*HalTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC$0$NUM_TIMERS; num++) 
    {
      /*HalTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC$0$Flags_t *flags = &/*HalTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC$0$m_flags[num];

      if (flags->isrunning) 
        {






          /*HalTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC$0$Timer_t *timer = &/*HalTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC$0$m_timers[num];
          int32_t elapsed = then - timer->t0;
          int32_t remaining = timer->dt - elapsed;
          bool compute_min_remaining = TRUE;






          if (elapsed >= 0 && timer->dt <= (uint32_t )elapsed) 
            {
              if (flags->isoneshot) 
                {
                  flags->isrunning = FALSE;
                  compute_min_remaining = FALSE;
                }
              else 
                {


                  timer->t0 += timer->dt;
                  remaining += timer->dt;
                }

              /*HalTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC$0$Timer$fired(num);
            }



          if (compute_min_remaining && flags->isrunning) 
            {
              if (remaining < 0) {
                min_remaining = 0;
                }
              else {
#line 123
                if (remaining < min_remaining) {
                  min_remaining = remaining;
                  }
                }
#line 125
              min_remaining_isset = TRUE;
            }
        }
    }

  if (min_remaining_isset) 
    {
      uint32_t now = /*HalTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC$0$TimerFrom$getNow();
      uint32_t elapsed = now - then;

#line 134
      if (min_remaining <= elapsed) {
        /*HalTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC$0$executeTimersNow$postTask();
        }
      else {
#line 137
        /*HalTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC$0$TimerFrom$startOneShotAt(now, min_remaining - elapsed);
        }
    }
}

# 449 "/Users/jingyuancheng/tinyos/moteiv/tos/platform/msp430/adc/MSP430ADC12M.nc"
static void MSP430ADC12M$RefVolt$isStable(RefVolt_t vref)
{
  if (MSP430ADC12M$vrefWait) {
      MSP430ADC12M$HPLADC12$startConversion();
      if (MSP430ADC12M$reserved & TIMER_USED) {
        MSP430ADC12M$startTimerA();
        }
#line 455
      MSP430ADC12M$reserved = ADC_IDLE;
      MSP430ADC12M$vrefWait = FALSE;
    }
}

# 73 "/Users/jingyuancheng/tinyos/moteiv/tos/platform/msp430/timer/MSP430TimerAExclusiveM.nc"
static result_t MSP430TimerAExclusiveM$TimerExclusive$startTimer(uint8_t rh)
{
  MSP430CompareControl_t ccSetSHI = { 
  .ccifg = 0, .cov = 0, .out = 1, .cci = 0, .ccie = 0, 
  .outmod = 0, .cap = 0, .clld = 0, .scs = 0, .ccis = 0, .cm = 0 };
  MSP430CompareControl_t ccResetSHI = { 
  .ccifg = 0, .cov = 0, .out = 0, .cci = 0, .ccie = 0, 
  .outmod = 0, .cap = 0, .clld = 0, .scs = 0, .ccis = 0, .cm = 0 };
  MSP430CompareControl_t ccRSOutmod = { 
  .ccifg = 0, .cov = 0, .out = 0, .cci = 0, .ccie = 0, 
  .outmod = 7, .cap = 0, .clld = 0, .scs = 0, .ccis = 0, .cm = 0 };

  if (!MSP430TimerAExclusiveM$ResourceValidate$validateUser(rh)) {
    return FAIL;
    }

  MSP430TimerAExclusiveM$ControlA1$setControl(ccResetSHI);
  MSP430TimerAExclusiveM$ControlA1$setControl(ccSetSHI);
  MSP430TimerAExclusiveM$ControlA1$setControl(ccResetSHI);
  MSP430TimerAExclusiveM$ControlA1$setControl(ccRSOutmod);
  MSP430TimerAExclusiveM$TimerA$setMode(MSP430TIMER_UP_MODE);
  return SUCCESS;
}

# 76 "/Users/jingyuancheng/tinyos/moteiv/tos/lib/resource/FcfsArbiterP.nc"
static bool /*MSP430ArbiterTimerAC.ArbiterC.ArbiterP*/FcfsArbiterP$0$ResourceValidate$validateUser(uint8_t rh)
#line 76
{
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 77
    {
      unsigned char __nesc_temp = 
#line 77
      rh != RESOURCE_NONE && rh == /*MSP430ArbiterTimerAC.ArbiterC.ArbiterP*/FcfsArbiterP$0$m_granted;

      {
#line 77
        __nesc_atomic_end(__nesc_atomic); 
#line 77
        return __nesc_temp;
      }
    }
#line 79
    __nesc_atomic_end(__nesc_atomic); }
}

# 79 "/Users/jingyuancheng/tinyos/moteiv/tos/platform/msp430/timer/MSP430TimerM.nc"
static void /*MSP430TimerC.MSP430TimerA*/MSP430TimerM$0$Timer$setMode(int mode)
{
  * (volatile uint16_t *)352U = (* (volatile uint16_t *)352U & ~(0x0020 | 0x0010)) | ((mode << 4) & (0x0020 | 0x0010));
}

# 161 "ADC8IO14P.nc"
static void ADC8IO14P$switchCtrl(void )
{

  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
    {
      if (ADC8IO14P$countCtrl == 0) {
        ADC8IO14P$Ctrl0$setHigh();
        }
      else {
#line 169
        ADC8IO14P$Ctrl0$setLow();
        }
      if (ADC8IO14P$countCtrl == 1) {
        ADC8IO14P$Ctrl1$setHigh();
        }
      else {
#line 174
        ADC8IO14P$Ctrl1$setLow();
        }
      if (ADC8IO14P$countCtrl == 2) {
        ADC8IO14P$Ctrl2$setHigh();
        }
      else {
#line 179
        ADC8IO14P$Ctrl2$setLow();
        }
      if (ADC8IO14P$countCtrl == 3) {
        ADC8IO14P$Ctrl3$setHigh();
        }
      else {
#line 184
        ADC8IO14P$Ctrl3$setLow();
        }
      if (ADC8IO14P$countCtrl == 4) {
        ADC8IO14P$Ctrl4$setHigh();
        }
      else {
#line 189
        ADC8IO14P$Ctrl4$setLow();
        }
      if (ADC8IO14P$countCtrl == 5) {
        ADC8IO14P$Ctrl5$setHigh();
        }
      else {
#line 194
        ADC8IO14P$Ctrl5$setLow();
        }
      if (ADC8IO14P$countCtrl == 6) {
        ADC8IO14P$Ctrl6$setHigh();
        }
      else {
#line 199
        ADC8IO14P$Ctrl6$setLow();
        }
      if (ADC8IO14P$countCtrl == 7) {
        ADC8IO14P$Ctrl7$setHigh();
        }
      else {
#line 204
        ADC8IO14P$Ctrl7$setLow();
        }
    }
#line 206
    __nesc_atomic_end(__nesc_atomic); }
}

# 184 "/Users/jingyuancheng/tinyos/moteiv/tos/platform/msp430/adc/MSP430ADC12M.nc"
static msp430ADCresult_t MSP430ADC12M$newRequest(uint8_t req, uint8_t num, void *dataDest, uint16_t length, uint16_t jiffies)
{
  bool access = FALSE;
  msp430ADCresult_t res = MSP430ADC12_FAIL;




  const int16_t num16 = num;



  bool interrupts_enabled = req & ADC_INTERRUPT_DISABLED ? 0 : 1;

#line 197
  req &= ~ADC_INTERRUPT_DISABLED;


  if (num16 >= 8U || (!MSP430ADC12M$reserved && (
  !length || (req == REPEAT_SEQUENCE_OF_CHANNELS && length > 16)))) {
    return MSP430ADC12_FAIL;
    }






  if (jiffies == 1 || jiffies == 2) {
    return MSP430ADC12_FAIL;
    }

  if (MSP430ADC12M$reserved & RESERVED) {
    if (!(MSP430ADC12M$reserved & VREF_WAIT) && MSP430ADC12M$owner == num16 && MSP430ADC12M$cmode == req) {
        MSP430ADC12M$HPLADC12$startConversion();
        if (MSP430ADC12M$reserved & TIMER_USED) {
          MSP430ADC12M$startTimerA();
          }
#line 219
        MSP430ADC12M$reserved = ADC_IDLE;
        return MSP430ADC12_SUCCESS;
      }
    else {
#line 222
      return MSP430ADC12_FAIL;
      }
    }
#line 224
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 224
    {
      if (MSP430ADC12M$cmode == ADC_IDLE) {
          MSP430ADC12M$owner = num16;
          MSP430ADC12M$cmode = SEQUENCE_OF_CHANNELS;
          access = TRUE;
        }
    }
#line 230
    __nesc_atomic_end(__nesc_atomic); }

  if (access) {
      res = MSP430ADC12_SUCCESS;
      switch (MSP430ADC12M$getRefVolt(num16)) 
        {
          case MSP430ADC12_FAIL: 
            MSP430ADC12M$cmode = ADC_IDLE;
          res = MSP430ADC12_FAIL;
          break;
          case MSP430ADC12_DELAYED: 


            req |= RESERVED | VREF_WAIT;
          res = MSP430ADC12_DELAYED;
          MSP430ADC12M$vrefWait = TRUE;

          case MSP430ADC12_SUCCESS: 
            {
              int8_t i;
#line 249
              int8_t memctlsUsed = length;
              uint16_t mask = 1;
              adc12memctl_t lastMemctl = MSP430ADC12M$adc12settings[num16].memctl;
              uint16_t ctl0 = (0x0000 | 0x0010) & ~0x0080;
              adc12ctl1_t ctl1 = { .adc12busy = 0, .conseq = 1, 
              .adc12ssel = MSP430ADC12M$adc12settings[num16].clockSourceSHT, 
              .adc12div = MSP430ADC12M$adc12settings[num16].clockDivSHT, .issh = 0, .shp = 1, 
              .shs = 1, .cstartadd = 0 };

#line 257
              if (length > 16) {
                  ctl1.conseq = 3;
                  memctlsUsed = 16;
                }
              MSP430ADC12M$bufPtr = dataDest;
              MSP430ADC12M$bufLength = length;
              MSP430ADC12M$bufOffset = 0;


              MSP430ADC12M$HPLADC12$disableConversion();
              if (jiffies == 0) {
                  ctl0 = (0x0000 | 0x0010) | 0x0080;
                  ctl1.shs = 0;
                }
              for (i = 0; i < memctlsUsed - 1; i++) 
                MSP430ADC12M$HPLADC12$setMemControl(i, MSP430ADC12M$adc12settings[num16].memctl);
              lastMemctl.eos = 1;
              MSP430ADC12M$HPLADC12$setMemControl(i, lastMemctl);
              if (interrupts_enabled) {
                MSP430ADC12M$HPLADC12$setIEFlags(mask << i);
                }
#line 277
              MSP430ADC12M$HPLADC12$setControl0_IgnoreRef(* (adc12ctl0_t *)&ctl0);
              MSP430ADC12M$HPLADC12$setSHT(MSP430ADC12M$adc12settings[num16].sampleHoldTime);

              if (req & SINGLE_CHANNEL) {
                  ctl1.conseq = 0;
                  MSP430ADC12M$cmode = SINGLE_CHANNEL;
                }
              else {
#line 283
                if (req & REPEAT_SINGLE_CHANNEL) {
                    ctl1.conseq = 2;
                    MSP430ADC12M$cmode = REPEAT_SINGLE_CHANNEL;
                  }
                else {
#line 286
                  if (req & REPEAT_SEQUENCE_OF_CHANNELS) {
                      ctl1.conseq = 3;
                      MSP430ADC12M$cmode = REPEAT_SEQUENCE_OF_CHANNELS;
                    }
                  }
                }
#line 290
              MSP430ADC12M$HPLADC12$setControl1(ctl1);

              if (req & RESERVED) {

                  MSP430ADC12M$reserved = req;
                  if (jiffies != 0) {
                      if (MSP430ADC12M$prepareTimerA(jiffies, MSP430ADC12M$adc12settings[num16].clockSourceSAMPCON, MSP430ADC12M$adc12settings[num16].clockDivSAMPCON) == SUCCESS) {
                          MSP430ADC12M$reserved |= TIMER_USED;
                        }
                      else {

                          MSP430ADC12M$cmode = ADC_IDLE;
                          res = MSP430ADC12_FAIL;
                          break;
                        }
                    }
                }
              else 
#line 306
                {

                  MSP430ADC12M$HPLADC12$startConversion();
                  if (jiffies != 0) {
                      if (MSP430ADC12M$prepareTimerA(jiffies, MSP430ADC12M$adc12settings[num16].clockSourceSAMPCON, MSP430ADC12M$adc12settings[num16].clockDivSAMPCON) == SUCCESS) {
                          MSP430ADC12M$startTimerA();
                        }
                      else {

                          MSP430ADC12M$cmode = ADC_IDLE;
                          res = MSP430ADC12_FAIL;
                          break;
                        }
                    }
                }
              res = MSP430ADC12_SUCCESS;
              break;
            }
        }
    }
  return res;
}

# 74 "/Users/jingyuancheng/tinyos/moteiv/tos/platform/msp430/adc/RefVoltM.nc"
static result_t RefVoltM$RefVolt$get(RefVolt_t vref)
#line 74
{
  result_t result = SUCCESS;

  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 77
    {
      if (RefVoltM$semaCount == 0) {
          if (RefVoltM$HPLADC12$isBusy()) {
            result = FAIL;
            }
          else 
#line 81
            {
              if (RefVoltM$state == RefVoltM$REFERENCE_OFF) {
                RefVoltM$switchRefOn(vref);
                }
              else {
#line 84
                if ((RefVoltM$state == RefVoltM$REFERENCE_1_5V_PENDING && vref == REFERENCE_2_5V) || (
                RefVoltM$state == RefVoltM$REFERENCE_2_5V_PENDING && vref == REFERENCE_1_5V)) {
                  RefVoltM$switchToRefPending(vref);
                  }
                else {
#line 87
                  if ((RefVoltM$state == RefVoltM$REFERENCE_1_5V_STABLE && vref == REFERENCE_2_5V) || (
                  RefVoltM$state == RefVoltM$REFERENCE_2_5V_STABLE && vref == REFERENCE_1_5V)) {
                    RefVoltM$switchToRefStable(vref);
                    }
                  }
                }
#line 90
              RefVoltM$semaCount++;
              RefVoltM$switchOff = FALSE;
              result = SUCCESS;
            }
        }
      else {

        if ((((
#line 95
        RefVoltM$state == RefVoltM$REFERENCE_1_5V_PENDING && vref == REFERENCE_1_5V) || (
        RefVoltM$state == RefVoltM$REFERENCE_2_5V_PENDING && vref == REFERENCE_2_5V)) || (
        RefVoltM$state == RefVoltM$REFERENCE_1_5V_STABLE && vref == REFERENCE_1_5V)) || (
        RefVoltM$state == RefVoltM$REFERENCE_2_5V_STABLE && vref == REFERENCE_2_5V)) {
            RefVoltM$semaCount++;
            RefVoltM$switchOff = FALSE;
            result = SUCCESS;
          }
        else {
#line 103
          result = FAIL;
          }
        }
    }
#line 106
    __nesc_atomic_end(__nesc_atomic); }
#line 105
  return result;
}

# 73 "/Users/jingyuancheng/tinyos/moteiv/tos/platform/msp430/adc/HPLADC12M.nc"
static void HPLADC12M$HPLADC12$setMemControl(uint8_t i, adc12memctl_t memControl)
#line 73
{
  uint8_t *memCtlPtr = (uint8_t *)(char *)0x0080;

#line 75
  if (i < 16) {
      memCtlPtr += i;
      *memCtlPtr = * (uint8_t *)&memControl;
    }
}

# 169 "/Users/jingyuancheng/tinyos/moteiv/tos/platform/msp430/adc/MSP430ADC12M.nc"
static result_t MSP430ADC12M$prepareTimerA(uint16_t interval, uint16_t csSAMPCON, uint16_t cdSAMPCON)
{
  if (MSP430ADC12M$rh == RESOURCE_NONE) {
    MSP430ADC12M$rh = MSP430ADC12M$Resource$immediateRequest(RESOURCE_NONE);
    }
#line 173
  if (MSP430ADC12M$rh != RESOURCE_NONE) {
      return MSP430ADC12M$TimerExclusive$prepareTimer(MSP430ADC12M$rh, interval, csSAMPCON, cdSAMPCON);
    }
  return FAIL;
}

# 23 "/Users/jingyuancheng/tinyos/moteiv/tos/platform/msp430/resource/MSP430ResourceConfigTimerAP.nc"
static void MSP430ResourceConfigTimerAP$idle(void )
#line 23
{
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 24
    MSP430ResourceConfigTimerAP$TimerA$setMode(MSP430TIMER_STOP_MODE);
#line 24
    __nesc_atomic_end(__nesc_atomic); }
}

# 104 "/Users/jingyuancheng/tinyos/moteiv/tos/platform/msp430/timer/MSP430TimerM.nc"
static void /*MSP430TimerC.MSP430TimerA*/MSP430TimerM$0$Timer$setClockSource(uint16_t clockSource)
{
  * (volatile uint16_t *)352U = (* (volatile uint16_t *)352U & ~(256U | 512U)) | ((clockSource << 8) & (256U | 512U));
}

static void /*MSP430TimerC.MSP430TimerA*/MSP430TimerM$0$Timer$setInputDivider(uint16_t inputDivider)
{
  * (volatile uint16_t *)352U = (* (volatile uint16_t *)352U & ~(0x0040 | 0x0080)) | ((inputDivider << 6) & (0x0040 | 0x0080));
}

# 100 "/Users/jingyuancheng/tinyos/moteiv/tos/lib/timer/TransformAlarmC.nc"
static void /*HalTimerMilliC.AlarmMilliC.Transform*/TransformAlarmC$0$Alarm$startAt(/*HalTimerMilliC.AlarmMilliC.Transform*/TransformAlarmC$0$to_size_type t0, /*HalTimerMilliC.AlarmMilliC.Transform*/TransformAlarmC$0$to_size_type dt)
{
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
    {
      /*HalTimerMilliC.AlarmMilliC.Transform*/TransformAlarmC$0$m_t0 = t0;
      /*HalTimerMilliC.AlarmMilliC.Transform*/TransformAlarmC$0$m_dt = dt;
      /*HalTimerMilliC.AlarmMilliC.Transform*/TransformAlarmC$0$set_alarm();
    }
#line 107
    __nesc_atomic_end(__nesc_atomic); }
}

# 105 "/Users/jingyuancheng/tinyos/moteiv/tos/lib/resource/FcfsArbiterP.nc"
static void /*MSP430ArbiterTimerAC.ArbiterC.ArbiterP*/FcfsArbiterP$0$Resource$release(uint8_t id)
#line 105
{
  uint8_t grantMode = 0;

  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 108
    {
      if (id == /*MSP430ArbiterTimerAC.ArbiterC.ArbiterP*/FcfsArbiterP$0$m_granted) {
          uint8_t granted = rqueue_pop(/*MSP430ArbiterTimerAC.ArbiterC.ArbiterP*/FcfsArbiterP$0$queue());

          if (granted == RQUEUE_NONE) {
              /*MSP430ArbiterTimerAC.ArbiterC.ArbiterP*/FcfsArbiterP$0$m_granted = RESOURCE_NONE;
              grantMode = 2;
            }
          else {
              /*MSP430ArbiterTimerAC.ArbiterC.ArbiterP*/FcfsArbiterP$0$m_granted = granted;
              if (/*MSP430ArbiterTimerAC.ArbiterC.ArbiterP*/FcfsArbiterP$0$m_urgentCount > 0) {
                  /*MSP430ArbiterTimerAC.ArbiterC.ArbiterP*/FcfsArbiterP$0$m_urgentCount--;
                  grantMode = 1;
                }
            }
        }
      else {
          {
#line 125
            __nesc_atomic_end(__nesc_atomic); 
#line 125
            return;
          }
        }
    }
#line 128
    __nesc_atomic_end(__nesc_atomic); }
  if (grantMode == 0) {
#line 129
    /*MSP430ArbiterTimerAC.ArbiterC.ArbiterP*/FcfsArbiterP$0$GrantTask$postTask();
    }
  else {
#line 130
    if (grantMode == 1) {
#line 130
      /*MSP430ArbiterTimerAC.ArbiterC.ArbiterP*/FcfsArbiterP$0$GrantTask$postUrgentTask();
      }
    else {
#line 131
      /*MSP430ArbiterTimerAC.ArbiterC.ArbiterP*/FcfsArbiterP$0$Arbiter$idle();
      }
    }
}

# 49 "/Users/jingyuancheng/tinyos/moteiv/tos/lib/sched/TaskBasic.nc"
static result_t UartPresenceM$taskConnected$postTask(void ){
#line 49
  unsigned char __nesc_result;
#line 49

#line 49
  __nesc_result = SchedulerBasicP$TaskBasic$postTask(UartPresenceM$taskConnected);
#line 49

#line 49
  return __nesc_result;
#line 49
}
#line 49
# 98 "/Users/jingyuancheng/tinyos/moteiv/tos/platform/msp430/adc/MSP430ADC12M.nc"
static result_t MSP430ADC12M$ADCSingle$bind(uint8_t num, MSP430ADC12Settings_t settings)
{
  result_t res = FAIL;
  adc12memctl_t memctl = { .inch = settings.inputChannel, 
  .sref = settings.referenceVoltage, 
  .eos = 0 };

  if (num >= 8U) {
    return FAIL;
    }
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
    {
      if (MSP430ADC12M$cmode == ADC_IDLE || MSP430ADC12M$owner != num) {
          MSP430ADC12M$configureAdcPin(settings.inputChannel);
          MSP430ADC12M$adc12settings[num].refVolt2_5 = settings.refVolt2_5;
          MSP430ADC12M$adc12settings[num].gotRefVolt = 0;
          MSP430ADC12M$adc12settings[num].clockSourceSHT = settings.clockSourceSHT;
          MSP430ADC12M$adc12settings[num].clockSourceSAMPCON = settings.clockSourceSAMPCON;
          MSP430ADC12M$adc12settings[num].clockDivSAMPCON = settings.clockDivSAMPCON;
          MSP430ADC12M$adc12settings[num].clockDivSHT = settings.clockDivSHT;
          MSP430ADC12M$adc12settings[num].sampleHoldTime = settings.sampleHoldTime;
          MSP430ADC12M$adc12settings[num].memctl = memctl;
          res = SUCCESS;
        }
    }
#line 122
    __nesc_atomic_end(__nesc_atomic); }
  return res;
}

# 55 "/Users/jingyuancheng/tinyos/moteiv/tos/platform/msp430/HPLUSART1M.nc"
 __attribute((wakeup)) __attribute((interrupt(6))) void sig_UART1RX_VECTOR(void )
#line 55
{
  uint8_t temp = U1RXBUF;

#line 57
  HPLUSART1M$USARTData$rxDone(temp);
}

 __attribute((wakeup)) __attribute((interrupt(4))) void sig_UART1TX_VECTOR(void )
#line 60
{
  HPLUSART1M$USARTData$txDone();
}

# 495 "/Users/jingyuancheng/tinyos/moteiv/tos/platform/tmote/FramerP.nc"
static result_t FramerP$TxArbitraryByte(uint8_t inByte)
#line 495
{
  if (inByte == FramerP$HDLC_FLAG_BYTE || inByte == FramerP$HDLC_CTLESC_BYTE) {
      /* atomic removed: atomic calls only */
#line 497
      {
        FramerP$gPrevTxState = FramerP$gTxState;
        FramerP$gTxState = FramerP$TXSTATE_ESC;
        FramerP$gTxEscByte = inByte;
      }
      inByte = FramerP$HDLC_CTLESC_BYTE;
    }

  return FramerP$ByteComm$txByte(inByte);
}

# 59 "/Users/jingyuancheng/tinyos/moteiv/tos/platform/msp430/MSP430InterruptM.nc"
 __attribute((wakeup)) __attribute((interrupt(8))) void sig_PORT1_VECTOR(void )
{
  volatile int n = MSP430InterruptM$P1IFG & MSP430InterruptM$P1IE;

  if (n & (1 << 0)) {
#line 63
      MSP430InterruptM$Port10$fired();
#line 63
      return;
    }
#line 64
  if (n & (1 << 1)) {
#line 64
      MSP430InterruptM$Port11$fired();
#line 64
      return;
    }
#line 65
  if (n & (1 << 2)) {
#line 65
      MSP430InterruptM$Port12$fired();
#line 65
      return;
    }
#line 66
  if (n & (1 << 3)) {
#line 66
      MSP430InterruptM$Port13$fired();
#line 66
      return;
    }
#line 67
  if (n & (1 << 4)) {
#line 67
      MSP430InterruptM$Port14$fired();
#line 67
      return;
    }
#line 68
  if (n & (1 << 5)) {
#line 68
      MSP430InterruptM$Port15$fired();
#line 68
      return;
    }
#line 69
  if (n & (1 << 6)) {
#line 69
      MSP430InterruptM$Port16$fired();
#line 69
      return;
    }
#line 70
  if (n & (1 << 7)) {
#line 70
      MSP430InterruptM$Port17$fired();
#line 70
      return;
    }
}

 __attribute((wakeup)) __attribute((interrupt(2))) void sig_PORT2_VECTOR(void )
{
  volatile int n = MSP430InterruptM$P2IFG & MSP430InterruptM$P2IE;

  if (n & (1 << 0)) {
#line 78
      MSP430InterruptM$Port20$fired();
#line 78
      return;
    }
#line 79
  if (n & (1 << 1)) {
#line 79
      MSP430InterruptM$Port21$fired();
#line 79
      return;
    }
#line 80
  if (n & (1 << 2)) {
#line 80
      MSP430InterruptM$Port22$fired();
#line 80
      return;
    }
#line 81
  if (n & (1 << 3)) {
#line 81
      MSP430InterruptM$Port23$fired();
#line 81
      return;
    }
#line 82
  if (n & (1 << 4)) {
#line 82
      MSP430InterruptM$Port24$fired();
#line 82
      return;
    }
#line 83
  if (n & (1 << 5)) {
#line 83
      MSP430InterruptM$Port25$fired();
#line 83
      return;
    }
#line 84
  if (n & (1 << 6)) {
#line 84
      MSP430InterruptM$Port26$fired();
#line 84
      return;
    }
#line 85
  if (n & (1 << 7)) {
#line 85
      MSP430InterruptM$Port27$fired();
#line 85
      return;
    }
}

#line 88
 __attribute((wakeup)) __attribute((interrupt(28))) void sig_NMI_VECTOR(void )
{
  volatile int n = IFG1;

#line 91
  if (n & (1 << 4)) {
#line 91
      MSP430InterruptM$NMI$fired();
#line 91
      return;
    }
#line 92
  if (n & (1 << 1)) {
#line 92
      MSP430InterruptM$OF$fired();
#line 92
      return;
    }
#line 93
  if (FCTL3 & 0x0004) {
#line 93
      MSP430InterruptM$ACCV$fired();
#line 93
      return;
    }
}

# 58 "/Users/jingyuancheng/tinyos/moteiv/tos/platform/msp430/HPLUSART0M.nc"
 __attribute((wakeup)) __attribute((interrupt(18))) void sig_UART0RX_VECTOR(void )
#line 58
{
  uint8_t temp = U0RXBUF;

#line 60
  HPLUSART0M$USARTData$rxDone(temp);
}

 __attribute((wakeup)) __attribute((interrupt(16))) void sig_UART0TX_VECTOR(void )
#line 63
{
  if (HPLUSART0M$USARTControl$isI2C()) {
    HPLUSART0M$HPLI2CInterrupt$fired();
    }
  else {
#line 67
    HPLUSART0M$USARTData$txDone();
    }
}

# 163 "/Users/jingyuancheng/tinyos/moteiv/tos/platform/msp430/adc/HPLADC12M.nc"
 __attribute((wakeup)) __attribute((interrupt(14))) void sig_ADC_VECTOR(void )
#line 163
{
  uint16_t iv = HPLADC12M$ADC12IV;

#line 165
  switch (iv) 
    {
      case 2: HPLADC12M$HPLADC12$memOverflow();
#line 167
      return;
      case 4: HPLADC12M$HPLADC12$timeOverflow();
#line 168
      return;
    }
  iv >>= 1;
  if (iv && iv < 19) {
    HPLADC12M$HPLADC12$converted(iv - 3);
    }
}

# 460 "/Users/jingyuancheng/tinyos/moteiv/tos/platform/msp430/adc/MSP430ADC12M.nc"
static void MSP430ADC12M$stopConversion(void )
{
  /* atomic removed: atomic calls only */
#line 462
  {
    MSP430ADC12M$TimerExclusive$stopTimer(MSP430ADC12M$rh);
    MSP430ADC12M$Resource$release();
    MSP430ADC12M$rh = RESOURCE_NONE;
  }
  MSP430ADC12M$HPLADC12$stopConversion();
  MSP430ADC12M$HPLADC12$setIEFlags(0);
  MSP430ADC12M$HPLADC12$resetIFGs();
  if (MSP430ADC12M$adc12settings[MSP430ADC12M$owner].gotRefVolt) {
    MSP430ADC12M$releaseRefVolt(MSP430ADC12M$owner);
    }
#line 472
  MSP430ADC12M$cmode = ADC_IDLE;
}

# 98 "/Users/jingyuancheng/tinyos/moteiv/tos/platform/msp430/adc/HPLADC12M.nc"
static void HPLADC12M$HPLADC12$resetIFGs(void )
#line 98
{

  if (!HPLADC12M$ADC12IFG) {
    return;
    }
  else 
#line 102
    {
      uint8_t i;
      volatile uint16_t mud;

#line 105
      for (i = 0; i < 16; i++) 
        mud = HPLADC12M$HPLADC12$getMem(i);
    }
}

# 439 "/Users/jingyuancheng/tinyos/moteiv/tos/platform/msp430/adc/MSP430ADC12M.nc"
static result_t MSP430ADC12M$ADCSingle$default$dataReady(uint8_t num, uint16_t data)
{
  return FAIL;
}

# 133 "/Users/jingyuancheng/tinyos/moteiv/tos/platform/msp430/adc/MSP430ADC12Single.nc"
static result_t MSP430ADC12M$ADCSingle$dataReady(uint8_t arg_0x102dc9578, uint16_t data){
#line 133
  unsigned char __nesc_result;
#line 133

#line 133
  switch (arg_0x102dc9578) {
#line 133
    case 0U:
#line 133
      __nesc_result = ADC8IO14P$ADC0$dataReady(data);
#line 133
      break;
#line 133
    case 1U:
#line 133
      __nesc_result = ADC8IO14P$ADC1$dataReady(data);
#line 133
      break;
#line 133
    case 2U:
#line 133
      __nesc_result = ADC8IO14P$ADC2$dataReady(data);
#line 133
      break;
#line 133
    case 3U:
#line 133
      __nesc_result = ADC8IO14P$ADC3$dataReady(data);
#line 133
      break;
#line 133
    case 4U:
#line 133
      __nesc_result = ADC8IO14P$ADC4$dataReady(data);
#line 133
      break;
#line 133
    case 5U:
#line 133
      __nesc_result = ADC8IO14P$ADC5$dataReady(data);
#line 133
      break;
#line 133
    case 6U:
#line 133
      __nesc_result = ADC8IO14P$ADC6$dataReady(data);
#line 133
      break;
#line 133
    case 7U:
#line 133
      __nesc_result = ADC8IO14P$ADC7$dataReady(data);
#line 133
      break;
#line 133
    default:
#line 133
      __nesc_result = MSP430ADC12M$ADCSingle$default$dataReady(arg_0x102dc9578, data);
#line 133
      break;
#line 133
    }
#line 133

#line 133
  return __nesc_result;
#line 133
}
#line 133
