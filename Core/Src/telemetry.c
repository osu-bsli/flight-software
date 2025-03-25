#include "telemetry.h"
#include "string.h"
#include "checksum.h"

void telemetry_packet_make_header(struct telemetry_packet *p)
{
  // Copy in "FUCKPETER" magic
  memcpy(p->magic, TELEMETRY_PACKET_MAGIC, sizeof(p->magic));
  p->size = sizeof(struct telemetry_packet);

  // Zero out the CRC16 field
  p->crc16 = 0;

  // Write CRC
  p->crc16 = crc_modbus((const unsigned char*)p, sizeof(struct telemetry_packet));
}
