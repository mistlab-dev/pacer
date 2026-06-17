package main

import (
	"encoding/binary"
	"flag"
	"fmt"
	"log"
	"math"
	"os"
	"os/signal"
	"syscall"
	"time"

	"go.bug.st/serial"
)

// 遥控帧协议: 0xAA 0x55 + throttle(float32) + roll(float32) + pitch(float32) + yaw(float32)
// 共 18 字节, 小端序

const (
	frameHeader1 = 0xAA
	frameHeader2 = 0x55
	frameSize    = 18
)

// RemoteCmd 遥控命令
type RemoteCmd struct {
	Throttle float32 // 油门 0~1
	Roll     float32 // 滚转 -1~+1 (左负右正)
	Pitch    float32 // 俯仰 -1~+1 (前负后正)
	Yaw      float32 // 偏航 -1~+1 (左负右正)
}

// Encode 编码为 18 字节帧
func (c *RemoteCmd) Encode() []byte {
	buf := make([]byte, frameSize)
	buf[0] = frameHeader1
	buf[1] = frameHeader2
	binary.LittleEndian.PutUint32(buf[2:6], math.Float32bits(c.Throttle))
	binary.LittleEndian.PutUint32(buf[6:10], math.Float32bits(c.Roll))
	binary.LittleEndian.PutUint32(buf[10:14], math.Float32bits(c.Pitch))
	binary.LittleEndian.PutUint32(buf[14:18], math.Float32bits(c.Yaw))
	return buf
}

// RemoteController 遥控控制器
type RemoteController struct {
	port     serial.Port
	portName string
	baudRate int
	cmd      RemoteCmd
	running  bool
}

// NewRemoteController 创建遥控器
func NewRemoteController(portName string, baudRate int) (*RemoteController, error) {
	mode := &serial.Mode{
		BaudRate: baudRate,
	}
	port, err := serial.Open(portName, mode)
	if err != nil {
		return nil, fmt.Errorf("open serial port: %w", err)
	}

	return &RemoteController{
		port:     port,
		portName: portName,
		baudRate: baudRate,
		cmd:      RemoteCmd{Throttle: 0, Roll: 0, Pitch: 0, Yaw: 0},
		running:  true,
	}, nil
}

// Close 关闭串口
func (rc *RemoteController) Close() {
	rc.running = false
	if rc.port != nil {
		rc.port.Close()
	}
}

// SendCmd 发送遥控命令
func (rc *RemoteController) SendCmd() error {
	frame := rc.cmd.Encode()
	n, err := rc.port.Write(frame)
	if err != nil {
		return fmt.Errorf("write frame: %w", err)
	}
	if n != frameSize {
		return fmt.Errorf("write incomplete: %d/%d", n, frameSize)
	}
	return nil
}

// SetThrottle 设置油门 (0~1)
func (rc *RemoteController) SetThrottle(t float32) {
	rc.cmd.Throttle = clamp(t, 0, 1)
}

// SetRoll 设置滚转 (-1~+1)
func (rc *RemoteController) SetRoll(r float32) {
	rc.cmd.Roll = clamp(r, -1, 1)
}

// SetPitch 设置俯仰 (-1~+1)
func (rc *RemoteController) SetPitch(p float32) {
	rc.cmd.Pitch = clamp(p, -1, 1)
}

// SetYaw 设置偏航 (-1~+1)
func (rc *RemoteController) SetYaw(y float32) {
	rc.cmd.Yaw = clamp(y, -1, 1)
}

// Reset 重置所有通道为零
func (rc *RemoteController) Reset() {
	rc.cmd = RemoteCmd{Throttle: 0, Roll: 0, Pitch: 0, Yaw: 0}
}

// clamp 浮点数限幅
func clamp(v, min, max float32) float32 {
	if v < min {
		return min
	}
	if v > max {
		return max
	}
	return v
}

// RunLoop 定时发送循环 (默认 50Hz)
func (rc *RemoteController) RunLoop(freq int) {
 interval := time.Duration(1000/freq) * time.Millisecond

 ticker := time.NewTicker(interval)
 defer ticker.Stop()

 for rc.running {
  select {
  case <-ticker.C:
   if err := rc.SendCmd(); err != nil {
    log.Printf("send error: %v", err)
   }
  }
 }
}

func main() {
 portName := flag.String("port", "", "serial port device (e.g. COM3 on Windows, /dev/ttyUSB0 on Linux)")
 baudRate := flag.Int("baud", 115200, "serial baud rate")
 freq := flag.Int("freq", 50, "send frequency Hz")
 throttle := flag.Float64("t", 0, "initial throttle 0~1")
 roll := flag.Float64("r", 0, "initial roll -1~1")
 pitch := flag.Float64("p", 0, "initial pitch -1~1")
 yaw := flag.Float64("y", 0, "initial yaw -1~1")
 demo := flag.Bool("demo", false, "run demo mode (auto oscillate)")
 flag.Parse()

 if *portName == "" {
  fmt.Println("Usage: pacer-remote -port <device> [options]")
  fmt.Println("\nOptions:")
  flag.PrintDefaults()
  fmt.Println("\nExamples:")
  fmt.Println("  Windows: pacer-remote -port COM3 -t 0.5")
  fmt.Println("  Linux:   pacer-remote -port /dev/ttyUSB0 -t 0.3 -r 0.1")
  fmt.Println("  Demo:    pacer-remote -port COM3 -demo")
  os.Exit(1)
 }

 rc, err := NewRemoteController(*portName, *baudRate)
 if err != nil {
  log.Fatalf("init remote controller: %v", err)
 }
 defer rc.Close()

 // 设置初始值
 rc.SetThrottle(float32(*throttle))
 rc.SetRoll(float32(*roll))
 rc.SetPitch(float32(*pitch))
 rc.SetYaw(float32(*yaw))

 log.Printf("Remote controller started: %s @ %d bps, %d Hz", *portName, *baudRate, *freq)
 log.Printf("Initial: T=%.2f R=%.2f P=%.2f Y=%.2f", *throttle, *roll, *pitch, *yaw)

 // 启动发送循环
 go rc.RunLoop(*freq)

 // Demo 模式: 自动摇摆
 if *demo {
  go runDemo(rc)
 }

 // 交互式控制
 runInteractive(rc)
}

// runDemo 演示模式 - 自动摇摆
func runDemo(rc *RemoteController) {
 t := 0.0
 for rc.running {
  time.Sleep(50 * time.Millisecond)
  t += 0.05

  // 油门保持 0.5
  rc.SetThrottle(0.5)

  // 滚转周期性摇摆 ±0.3
  rc.SetRoll(float32(0.3 * math.Sin(t)))

  // 俯仰周期性摇摆 ±0.2
  rc.SetPitch(float32(0.2 * math.Cos(t * 0.7)))
 }

 // 结束时归零
 rc.Reset()
}

// runInteractive 交互式键盘控制
func runInteractive(rc *RemoteController) {
 fmt.Println("\n=== Interactive Control ===")
 fmt.Println("Commands:")
 fmt.Println("  t+/t- : throttle ±0.05")
 fmt.Println("  r+/r- : roll ±0.1")
 fmt.Println("  p+/p- : pitch ±0.1")
 fmt.Println("  y+/y- : yaw ±0.1")
 fmt.Println("  z     : reset all to zero")
 fmt.Println("  q     : quit")
 fmt.Println("============================")

 sigCh := make(chan os.Signal, 1)
 signal.Notify(sigCh, syscall.SIGINT, syscall.SIGTERM)

 // 简化的命令读取 (实际应用可用 golang.org/x/term 实现真正的终端控制)
 for {
  select {
  case <-sigCh:
   fmt.Println("\nInterrupt received, shutting down...")
   rc.Reset()
   rc.SendCmd()
   rc.Close()
   return
  default:
   // 这里需要真正的键盘读取实现
   // 简化版只响应 Ctrl+C
   time.Sleep(100 * time.Millisecond)
  }
 }
}