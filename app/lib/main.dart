import 'dart:async';
import 'dart:typed_data';
import 'dart:ui' as ui;
import 'package:flutter/material.dart';
import 'package:web_socket_channel/web_socket_channel.dart';

void main() {
  runApp(const TcamApp());
}

class TcamApp extends StatelessWidget {
  const TcamApp({super.key});

  @override
  Widget build(BuildContext context) {
    return MaterialApp(
      title: 'Trail Camera',
      theme: ThemeData.dark(useMaterial3: true),
      home: const ViewerPage(),
    );
  }
}

class ViewerPage extends StatefulWidget {
  const ViewerPage({super.key});

  @override
  State<ViewerPage> createState() => _ViewerPageState();
}

class _ViewerPageState extends State<ViewerPage> {
  static const _wsUrl = 'ws://192.168.4.1:80/stream';
  static const _frameW = 640;
  static const _frameH = 480;
  static const _headerLen = 5;

  WebSocketChannel? _channel;
  bool _connected = false;
  int _frameCount = 0;
  int _position = 0;
  int _seq = 0;
  ui.Image? _image;
  String _status = 'Disconnected';
  DateTime? _lastFrame;
  double _fps = 0;

  @override
  void initState() {
    super.initState();
    _connect();
  }

  @override
  void dispose() {
    _channel?.sink.close();
    super.dispose();
  }

  void _connect() {
    setState(() => _status = 'Connecting...');
    try {
      _channel = WebSocketChannel.connect(Uri.parse(_wsUrl));
      _channel!.stream.listen(
        _onData,
        onError: (e) {
          setState(() {
            _connected = false;
            _status = 'Error: $e';
          });
          _reconnect();
        },
        onDone: () {
          setState(() {
            _connected = false;
            _status = 'Disconnected';
          });
          _reconnect();
        },
      );
      setState(() {
        _connected = true;
        _status = 'Connected';
      });
    } catch (e) {
      setState(() => _status = 'Failed: $e');
      _reconnect();
    }
  }

  void _reconnect() {
    Future.delayed(const Duration(seconds: 2), () {
      if (mounted && !_connected) _connect();
    });
  }

  void _onData(dynamic data) {
    if (data is! List<int> && data is! Uint8List) return;
    final bytes = data is Uint8List ? data : Uint8List.fromList(data);
    if (bytes.length < _headerLen) return;

    final position = bytes[0];
    final seq = (bytes[1] << 24) | (bytes[2] << 16) | (bytes[3] << 8) | bytes[4];
    final pixels = bytes.sublist(_headerLen);

    if (pixels.length != _frameW * _frameH) return;

    _decodeGrayscale(pixels).then((img) {
      if (!mounted) return;
      final now = DateTime.now();
      if (_lastFrame != null) {
        final dt = now.difference(_lastFrame!).inMilliseconds;
        if (dt > 0) _fps = _fps * 0.8 + (1000.0 / dt) * 0.2;
      }
      _lastFrame = now;
      setState(() {
        _image = img;
        _position = position;
        _seq = seq;
        _frameCount++;
      });
    });
  }

  Future<ui.Image> _decodeGrayscale(Uint8List gray) {
    final rgba = Uint8List(_frameW * _frameH * 4);
    for (int i = 0; i < gray.length; i++) {
      final p = gray[i];
      final o = i * 4;
      rgba[o] = p;
      rgba[o + 1] = p;
      rgba[o + 2] = p;
      rgba[o + 3] = 255;
    }
    final completer = Completer<ui.Image>();
    ui.decodeImageFromPixels(
      rgba, _frameW, _frameH, ui.PixelFormat.rgba8888,
      (img) => completer.complete(img),
    );
    return completer.future;
  }

  static const _posNames = ['Left', 'Center', 'Right'];

  @override
  Widget build(BuildContext context) {
    return Scaffold(
      appBar: AppBar(
        title: const Text('Trail Camera'),
        actions: [
          Padding(
            padding: const EdgeInsets.symmetric(horizontal: 16),
            child: Center(
              child: Row(
                mainAxisSize: MainAxisSize.min,
                children: [
                  Icon(
                    _connected ? Icons.circle : Icons.circle_outlined,
                    size: 12,
                    color: _connected ? Colors.green : Colors.red,
                  ),
                  const SizedBox(width: 8),
                  Text(_status),
                ],
              ),
            ),
          ),
        ],
      ),
      body: Column(
        children: [
          Expanded(
            child: Center(
              child: _image != null
                  ? RawImage(image: _image, fit: BoxFit.contain)
                  : Text(
                      _connected ? 'Waiting for frames...' : _status,
                      style: Theme.of(context).textTheme.titleLarge,
                    ),
            ),
          ),
          Container(
            padding: const EdgeInsets.all(12),
            color: Theme.of(context).colorScheme.surfaceContainerHighest,
            child: Row(
              mainAxisAlignment: MainAxisAlignment.spaceAround,
              children: [
                _stat('Position', _frameCount > 0
                    ? (_position < _posNames.length
                        ? _posNames[_position]
                        : '$_position')
                    : '--'),
                _stat('Sequence', _frameCount > 0 ? '$_seq' : '--'),
                _stat('Frames', '$_frameCount'),
                _stat('FPS', _fps > 0 ? _fps.toStringAsFixed(1) : '--'),
              ],
            ),
          ),
        ],
      ),
      floatingActionButton: FloatingActionButton(
        onPressed: () {
          _channel?.sink.close();
          _connected = false;
          _connect();
        },
        tooltip: 'Reconnect',
        child: const Icon(Icons.refresh),
      ),
    );
  }

  Widget _stat(String label, String value) {
    return Column(
      mainAxisSize: MainAxisSize.min,
      children: [
        Text(value, style: const TextStyle(
          fontSize: 18, fontWeight: FontWeight.bold)),
        Text(label, style: TextStyle(
          fontSize: 12, color: Colors.grey[400])),
      ],
    );
  }
}
