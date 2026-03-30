// lora_packet.dart
// Parses SDD502 compressed LoRa packet format.
//
// Wire format:
//   Byte 0:     flags — mode[1:0] | position[3:2] | reserved[7:4]
//   Bytes 1–2:  scale_factor (uint16 big-endian)
//   Bytes 3–6:  bbox in block coords — x1, y1, x2, y2
//   Bytes 7…N:  active block bitmap (ceil(bbox_blocks / 8) bytes, MSB first)
//   Bytes N…M:  4-bit DC values, high nibble first, active blocks only
//               dc_real = nibble × scale_factor / 15  (per-pixel average delta)

import 'dart:typed_data';

enum CameraPosition { left, center, right }

class ActiveBlock {
  final int blockX;   // block column in frame (0–79 for 640px wide)
  final int blockY;   // block row in frame    (0–59 for 480px tall)
  final double dc;    // dequantized per-pixel average delta

  const ActiveBlock({
    required this.blockX,
    required this.blockY,
    required this.dc,
  });
}

class LoraPacket {
  static const int frameBlocksW = 80;  // 640 / 8
  static const int frameBlocksH = 60;  // 480 / 8

  final CameraPosition position;
  final int            mode;
  final int            scaleFactor;
  final int            bboxX1, bboxY1, bboxX2, bboxY2;
  final List<ActiveBlock> activeBlocks;

  const LoraPacket({
    required this.position,
    required this.mode,
    required this.scaleFactor,
    required this.bboxX1,
    required this.bboxY1,
    required this.bboxX2,
    required this.bboxY2,
    required this.activeBlocks,
  });

  static LoraPacket parse(Uint8List data) {
    // Byte 0: flags
    final flags      = data[0];
    final mode       = flags & 0x03;
    final posIdx     = (flags >> 2) & 0x03;
    final position   = CameraPosition.values[posIdx.clamp(0, 2)];

    // Bytes 1–2: scale_factor
    final scaleFactor = (data[1] << 8) | data[2];

    // Bytes 3–6: bounding box (block coordinates)
    final bboxX1 = data[3];
    final bboxY1 = data[4];
    final bboxX2 = data[5];
    final bboxY2 = data[6];

    final bboxW      = bboxX2 - bboxX1 + 1;
    final bboxH      = bboxY2 - bboxY1 + 1;
    final bboxBlocks = bboxW * bboxH;
    final bitmapBytes = (bboxBlocks + 7) >> 3;

    // Bytes 7…: bitmap — 1 bit per block, row-major, MSB first
    int offset = 7;
    final activeBlocks = <ActiveBlock>[];
    int dcCount = 0;

    // First pass: find active block positions and count them
    final activePending = <(int bx, int by)>[];
    for (int row = 0; row < bboxH; row++) {
      for (int col = 0; col < bboxW; col++) {
        final bitIdx  = row * bboxW + col;
        final byteIdx = bitIdx >> 3;
        final bitPos  = 7 - (bitIdx & 7);
        if ((data[offset + byteIdx] >> bitPos) & 1 == 1) {
          activePending.add((bboxX1 + col, bboxY1 + row));
          dcCount++;
        }
      }
    }
    offset += bitmapBytes;

    // Second pass: unpack 4-bit DC nibbles (high nibble first)
    final dcBytes = (dcCount + 1) >> 1;
    int activeIdx = 0;
    for (int i = 0; i < dcBytes && activeIdx < dcCount; i++) {
      final byte = data[offset + i];

      // High nibble
      final hiRaw = (byte >> 4) & 0x0F;
      final hi    = hiRaw * scaleFactor / 15.0;
      final (bxH, byH) = activePending[activeIdx++];
      activeBlocks.add(ActiveBlock(blockX: bxH, blockY: byH, dc: hi));

      // Low nibble
      if (activeIdx < dcCount) {
        final loRaw = byte & 0x0F;
        final lo    = loRaw * scaleFactor / 15.0;
        final (bxL, byL) = activePending[activeIdx++];
        activeBlocks.add(ActiveBlock(blockX: bxL, blockY: byL, dc: lo));
      }
    }

    return LoraPacket(
      position:    position,
      mode:        mode,
      scaleFactor: scaleFactor,
      bboxX1:      bboxX1,
      bboxY1:      bboxY1,
      bboxX2:      bboxX2,
      bboxY2:      bboxY2,
      activeBlocks: activeBlocks,
    );
  }
}
