/**
 * Vector display using the MCP4921 DAC on the teensy3.1.
 *
 * Based on: https://trmm.net/V.st
 *
 */
#include <SPI.h>
#include <TimeLib.h>
#include "DMAChannel.h"
#include "hershey_font.h"

// How often should a frame be drawn if we haven't receivded any serial
// data from MAME (in ms).
#define REFRESH_RATE 20000u

#define BRIGHT_SHIFT 2 // larger numbers == dimmer lines
#define NORMAL_SHIFT 2 // but we can control with Z axis

#define OFF_SHIFT 5  // smaller numbers == slower transits
#define OFF_DWELL0 0 // time to sit beam on before starting a transit
#define OFF_DWELL1 2 // time to sit before starting a transit
#define OFF_DWELL2 2 // time to sit after finishing a transit

#define REST_X 2048 // wait in the center of the screen
#define REST_Y 2048

//input brightness from 0 to X is mapped from normal to bright
#define BRIGHT_OFF 500     // "0 volts", relative to reference
#define BRIGHT_NORMAL 2500 // fairly bright
#define BRIGHT_BRIGHT 3200 // super bright

#define FLIP_X
#define FLIP_Y

//protocol flags from advancemame
#define FLAG_COMPLETE 0x0
#define FLAG_RGB 0x1
#define FLAG_XY 0x2
#define FLAG_EXIT 0x7
#define FLAG_FRAME 0x4

//pins
#define SS_PIN 10
#define SS2_PIN 6
#define SDI 11
#define SCK 13

#define DEBUG_PIN 8
#define DELAY_PIN 7
#define IO_PIN 5

#define MAX_PTS 3000
static unsigned rx_points;
static unsigned num_points;
static uint32_t points[MAX_PTS];

#define MOVETO (1 << 11)
#define LINETO (2 << 11)
#define BRIGHTTO (3 << 11)

static DMAChannel spi_dma;
#define SPI_DMA_MAX 4096
//#define SPI_DMA_MAX 2048
static uint32_t spi_dma_q[2][SPI_DMA_MAX];
static unsigned spi_dma_which;
static unsigned spi_dma_count;
static unsigned spi_dma_in_progress;
static unsigned spi_dma_cs; // which pins are we using for IO

#define SPI_DMA_CS_BEAM_ON 2
#define SPI_DMA_CS_BEAM_OFF 1

static int spi_dma_tx_append(uint16_t value)
{
  spi_dma_q[spi_dma_which][spi_dma_count++] = 0 | ((uint32_t)value) | (spi_dma_cs << 16) // enable the chip select line
      ;

  if (spi_dma_count == SPI_DMA_MAX)
    return 1;
  return 0;
}

static void spi_dma_tx()
{
  if (spi_dma_count == 0)
    return;

  digitalWriteFast(DELAY_PIN, 1);

  // add a EOQ to the last entry
  spi_dma_q[spi_dma_which][spi_dma_count - 1] |= (1 << 27);

  spi_dma.clearComplete();
  spi_dma.clearError();
  spi_dma.sourceBuffer(
      spi_dma_q[spi_dma_which],
      4 * spi_dma_count // in bytes, not thingies
  );

  spi_dma_which = !spi_dma_which;
  spi_dma_count = 0;

  SPI0_SR = 0xFF0F0000;
  SPI0_RSER = 0 | SPI_RSER_RFDF_RE | SPI_RSER_RFDF_DIRS | SPI_RSER_TFFF_RE | SPI_RSER_TFFF_DIRS;

  spi_dma.enable();
  spi_dma_in_progress = 1;
}

static int spi_dma_tx_complete()
{
  //cli();

  // if nothing is in progress, we're "complete"
  if (!spi_dma_in_progress)
  {
    //sei();
    return 1;
  }

  if (!spi_dma.complete())
  {
    //sei();
    return 0;
  }

  digitalWriteFast(DELAY_PIN, 0);

  spi_dma.clearComplete();
  spi_dma.clearError();

  // the DMA hardware lies; it is not actually complete
  delayMicroseconds(5);
  //sei();

  // we are done!
  SPI0_RSER = 0;
  SPI0_SR = 0xFF0F0000;
  spi_dma_in_progress = 0;
  return 1;
}

static void spi_dma_setup()
{
  spi_dma.disable();
  spi_dma.destination((volatile uint32_t &)SPI0_PUSHR);
  spi_dma.disableOnCompletion();
  spi_dma.triggerAtHardwareEvent(DMAMUX_SOURCE_SPI0_TX);
  spi_dma.transferSize(4); // write all 32-bits of PUSHR

  SPI.beginTransaction(SPISettings(20000000, MSBFIRST, SPI_MODE0));

  // configure the output on pin 10 for !SS0 from the SPI hardware
  // and pin 6 for !SS1.
  CORE_PIN10_CONFIG = PORT_PCR_DSE | PORT_PCR_MUX(2);
  CORE_PIN6_CONFIG = PORT_PCR_DSE | PORT_PCR_MUX(2);

  // configure the frame size for 16-bit transfers
  SPI0_CTAR0 |= 0xF << 27;

  // send something to get it started

  spi_dma_which = 0;
  spi_dma_count = 0;

  spi_dma_tx_append(0);
  spi_dma_tx_append(0);
  spi_dma_tx();
}

void rx_append(int x, int y, unsigned bright)
{
  // store the 12-bits of x and y, as well as 6 bits of brightness
  // (three in X and three in Y)
  points[rx_points++] = (bright << 24) | (x << 12) | y << 0;
}

void moveto(int x, int y)
{
  rx_append(x, y, 0);
}

void lineto(int x, int y)
{
  rx_append(x, y, 24); // normal brightness
}

void brightto(int x, int y)
{
  rx_append(x, y, 63); // max brightness
}

int draw_character(char c, int x, int y, int size)
{

  const hershey_char_t *const f = &hershey_simplex[c - ' '];
  int next_moveto = 1;

  for (int i = 0; i < f->count; i++)
  {
    int dx = f->points[2 * i + 0];
    int dy = f->points[2 * i + 1];
    if (dx == -1)
    {
      next_moveto = 1;
      continue;
    }

    dx = (dx * size) * 3 / 4;
    dy = (dy * size) * 3 / 4;

    if (next_moveto)
      moveto(x + dx, y + dy);
    else
      lineto(x + dx, y + dy);

    next_moveto = 0;
  }

  return (f->width * size) * 3 / 4;
}

void draw_string(const char *s, int x, int y, int size)
{
  while (*s)
  {
    char c = *s++;
    x += draw_character(c, x, y, size);
  }
}

static void draw_test_pattern()
{
  // fill in some points for test and calibration
  moveto(0, 0);
  lineto(1024, 0);
  lineto(1024, 1024);
  lineto(0, 1024);
  lineto(0, 0);

  // triangle
  moveto(4095, 0);
  lineto(4095 - 512, 0);
  lineto(4095 - 0, 512);
  lineto(4095, 0);

  // cross
  moveto(4095, 4095);
  lineto(4095 - 512, 4095);
  lineto(4095 - 512, 4095 - 512);
  lineto(4095, 4095 - 512);
  lineto(4095, 4095);

  moveto(0, 4095);
  lineto(512, 4095);
  lineto(0, 4095 - 512);
  lineto(512, 4095 - 512);
  lineto(0, 4095);

  moveto(2047, 2047 - 512);
  brightto(2047, 2047 + 512);

  moveto(2047 - 512, 2047);
  brightto(2047 + 512, 2047);

  // and a gradiant scale
  for (int i = 1; i < 63; i += 4)
  {
    moveto(1600, 2048 + i * 8);
    rx_append(1900, 2048 + i * 8, i);
  }

  // draw the sunburst pattern in the corner
  moveto(0, 0);
  for (unsigned j = 0, i = 0; j <= 1024; j += 128, i++)
  {
    if (i & 1)
    {
      moveto(1024, j);
      rx_append(0, 0, i * 7);
    }
    else
    {
      rx_append(1024, j, i * 7);
    }
  }

  moveto(0, 0);
  for (unsigned j = 0, i = 0; j < 1024; j += 128, i++)
  {
    if (i & 1)
    {
      moveto(j, 1024);
      rx_append(0, 0, i * 7);
    }
    else
    {
      rx_append(j, 1024, i * 7);
    }
  }

  draw_string("http://v.st/", 2048 - 450, 2048 + 600, 6);

  draw_string("Firmware built", 2100, 1900, 3);
  draw_string(__DATE__, 2100, 1830, 3);
  draw_string(__TIME__, 2100, 1760, 3);

  int y = 2400;
  const int line_size = 70;

  draw_string("ELECTROHOME", 2100, y, 3);
  y -= line_size;
#ifdef FLIP_X
  draw_string("FLIP_X", 2100, y, 3);
  y -= line_size;
#endif
#ifdef FLIP_Y
  draw_string("FLIP_Y", 2100, y, 3);
  y -= line_size;
#endif
#ifdef SWAP_XY
  draw_string("SWAP_XY", 2100, y, 3);
  y -= line_size;
#endif
}

void setup()
{

  Serial.begin(9600);
  
  pinMode(DELAY_PIN, OUTPUT);
  pinMode(DEBUG_PIN, OUTPUT);
  pinMode(IO_PIN, OUTPUT);

  digitalWrite(DELAY_PIN, 0);
  digitalWrite(DEBUG_PIN, 0);
  digitalWrite(IO_PIN, 0);

  digitalWrite(SS2_PIN, 0);

  pinMode(SS_PIN, OUTPUT);
  pinMode(SS2_PIN, OUTPUT);
  pinMode(SDI, OUTPUT);
  pinMode(SCK, OUTPUT);

  rx_points = 0;

  draw_test_pattern();
  num_points = rx_points;
  rx_points = 0;

#ifdef SLOW_SPI
  SPI.begin();
  SPI.setClockDivider(SPI_CLOCK_DIV2);
#else
  //spi4teensy3::init(0);
  SPI.begin();
  SPI.setClockDivider(SPI_CLOCK_DIV2);

  //DMASPI0.begin();
  //DMASPI0.start();
  spi_dma_setup();
#endif
}

static void mpc4921_write(
    int channel,
    uint16_t value)
{
  value &= 0x0FFF; // mask out just the 12 bits of data

#if 1
  // select the output channel, buffered, no gain
  value |= 0x7000 | (channel == 1 ? 0x8000 : 0x0000);
#else
  // select the output channel, unbuffered, no gain
  value |= 0x3000 | (channel == 1 ? 0x8000 : 0x0000);
#endif

#ifdef SLOW_SPI
  SPI.transfer((value >> 8) & 0xFF);
  SPI.transfer((value >> 0) & 0xFF);
#else
  if (spi_dma_tx_append(value) == 0)
    return;

  // wait for the previous line to finish
  while (!spi_dma_tx_complete())
    ;

  // now send this line, which swaps buffers
  spi_dma_tx();
#endif
}

// x and y position are in 12-bit range
static uint16_t x_pos;
static uint16_t y_pos;

#ifdef SWAP_XY
#define DAC_X_CHAN 1
#define DAC_Y_CHAN 0
#else
#define DAC_X_CHAN 0
#define DAC_Y_CHAN 1
#endif

static inline void goto_x(uint16_t x)
{
  x_pos = x;
#ifdef FLIP_X
  mpc4921_write(DAC_X_CHAN, 4095 - x);
#else
  mpc4921_write(DAC_X_CHAN, x);
#endif
}

static inline void goto_y(
    uint16_t y)
{
  y_pos = y;
#ifdef FLIP_Y
  mpc4921_write(DAC_Y_CHAN, 4095 - y);
#else
  mpc4921_write(DAC_Y_CHAN, y);
#endif
}

static void dwell(const int count)
{
  for (int i = 0; i < count; i++)
  {
    if (i & 1)
      goto_x(x_pos);
    else
      goto_y(y_pos);
  }
}

static inline void brightness(uint16_t bright)
{
  static unsigned last_bright;
  if (last_bright == bright)
    return;
  last_bright = bright;

  dwell(OFF_DWELL0);
  spi_dma_cs = SPI_DMA_CS_BEAM_OFF;

  // scale bright from OFF to BRIGHT
  if (bright > 64)
    bright = 64;

  int bright_scaled = BRIGHT_OFF;
  if (bright > 0)
    bright_scaled = BRIGHT_NORMAL + ((BRIGHT_BRIGHT - BRIGHT_NORMAL) * bright) / 64;

  mpc4921_write(1, bright_scaled);
  spi_dma_cs = SPI_DMA_CS_BEAM_ON;
}

static inline void _draw_lineto(int x1, int y1, const int bright_shift)
{
  int dx;
  int dy;
  int sx;
  int sy;

  const int x1_orig = x1;
  const int y1_orig = y1;

  int x_off = x1 & ((1 << bright_shift) - 1);
  int y_off = y1 & ((1 << bright_shift) - 1);
  x1 >>= bright_shift;
  y1 >>= bright_shift;
  int x0 = x_pos >> bright_shift;
  int y0 = y_pos >> bright_shift;

  goto_x(x_pos);
  goto_y(y_pos);

  if (x0 <= x1)
  {
    dx = x1 - x0;
    sx = 1;
  }
  else
  {
    dx = x0 - x1;
    sx = -1;
  }

  if (y0 <= y1)
  {
    dy = y1 - y0;
    sy = 1;
  }
  else
  {
    dy = y0 - y1;
    sy = -1;
  }

  int err = dx - dy;

  while (1)
  {
    if (x0 == x1 && y0 == y1)
      break;

    int e2 = 2 * err;
    if (e2 > -dy)
    {
      err = err - dy;
      x0 += sx;
      goto_x(x_off + (x0 << bright_shift));
    }
    if (e2 < dx)
    {
      err = err + dx;
      y0 += sy;
      goto_y(y_off + (y0 << bright_shift));
    }
  }

  // ensure that we end up exactly where we want
  goto_x(x1_orig);
  goto_y(y1_orig);
}

void draw_lineto(int x1, int y1, unsigned bright)
{
  brightness(bright);
  _draw_lineto(x1, y1, NORMAL_SHIFT);
}

void draw_moveto(int x1, int y1)
{
  brightness(0);

  // hold the current position for a few clocks
  // with the beam off
  dwell(OFF_DWELL1);
  _draw_lineto(x1, y1, OFF_SHIFT);
  dwell(OFF_DWELL2);
}

//return 1 when frame is complete, otherwise return 0
static int read_data()
{
  static uint32_t cmd = 0;
  static int frame_offset = 0;
  static uint8_t brightness = 0;

  uint8_t c = Serial.read();

  cmd = cmd << 8 | c;
  frame_offset++;
  if (frame_offset < 4)
    return 0;

  frame_offset = 0;

  uint8_t header = (cmd >> 29) & 0b00000111;

  //common case first
  if (header == FLAG_XY)
  {
    uint32_t y = (cmd >> 0) & 0x3fff;
    uint32_t x = (cmd >> 14) & 0x3fff;

    //blanking flag
    if ((cmd >> 28) & 0x01)
      rx_append(x, y, 0);
    else
      rx_append(x, y, brightness);
  }
  else if (header == FLAG_RGB)
  {
    // pick the max brightness from r g and b
    brightness = max(max((cmd >> 0) & 0xFF, (cmd >> 8) & 0xFF), (cmd >> 16) & 0xFF);
    // 8 bit to 6 bit brightness conversion
    brightness = brightness >> 2;
  }

  else if (header == FLAG_FRAME)
  {
    uint32_t frame_complexity = cmd & 0b0001111111111111111111111111111;
    // TODO: Use frame_complexity to adjust screen writing algorithms
  }
  else if (header == FLAG_COMPLETE)
  {
    num_points = rx_points;
    rx_points = 0;
    return 1; //start rendering!
  }

  return 0;
}

void loop()
{

  static uint32_t frame_micros;
  uint32_t now;

  while (1)
  {
    now = micros();

    // make sure we flush the partial buffer
    // once the last one has completed
    if (spi_dma_tx_complete())
    {
      if (rx_points == 0 && now - frame_micros > REFRESH_RATE)
        break;
      spi_dma_tx();
    }

    // start redraw when read_data is done
    if (Serial.available() && read_data() == 1)
      break;
  }

  frame_micros = now;

  // if there are any DMAs currently in transit, wait for them
  // to complete.
  while (!spi_dma_tx_complete())
    ;

  // now start any last buffered ones and wait for those
  // to complete.
  spi_dma_tx();
  while (!spi_dma_tx_complete())
    ;

  // flag that we have started an output frame
  digitalWriteFast(DEBUG_PIN, 1);

  // force a reference voltage write on every cycle
  spi_dma_cs = SPI_DMA_CS_BEAM_OFF;
  mpc4921_write(0, 2048);
  spi_dma_cs = SPI_DMA_CS_BEAM_ON;

  for (unsigned n = 0; n < num_points; n++)
  {
    const uint32_t pt = points[n];
    uint16_t x = (pt >> 12) & 0xFFF;
    uint16_t y = (pt >> 0) & 0xFFF;
    unsigned intensity = (pt >> 24) & 0x3F;

    if (intensity == 0)
      draw_moveto(x, y);
    else
      draw_lineto(x, y, intensity);
  }

  // go to the center of the screen, turn the beam off
  brightness(0);

  goto_x(REST_X);
  goto_y(REST_Y);

  // the USB loop above will flush eventually
  digitalWriteFast(DEBUG_PIN, 0);
}
