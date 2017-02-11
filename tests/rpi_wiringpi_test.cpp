#include <iostream>
#include <errno.h>
#include <wiringPiSPI.h>
#include <unistd.h>

using namespace std;

// channel is the wiringPi name for the chip select (or chip enable) pin.
// Set this to 0 or 1, depending on how it's connected.
static const int CHANNEL = 1;

int main()
{
  cout << "Initializing" << endl ;

  int fd = wiringPiSPISetup(CHANNEL, 500000);

  cout << "Init result: " << fd << endl;

  uint8_t send_data[1] = {0x23};
  wiringPiSPIDataRW(CHANNEL,send_data, 1);
  printf("Sent to SPI: 0x%02X. Read back from SPI: 0x%02X.\n", send_data[0], 0x23);
  if (send_data[0] != 0x23)
    printf("Do you have the loopback from MOSI to MISO connected?\n");

}
