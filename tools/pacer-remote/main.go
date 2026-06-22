package main

import (
	"bufio"
	"encoding/binary"
	"flag"
	"fmt"
	"log"
	"math"
	"os"
	"os/signal"
	"strings"
	"syscall"
	"time"

	"go.bug.st/serial"
)

// 遥控帧协议 v2: 0xAA 0x55 + 4×float32 + XOR 校验
// 共 19 字节, 小端序

const (
	frameHeader1 = 0xAA
	frameHeader2 = 0x55
	frameSize    = 19
)

// RemoteCmd 遥控命令
type RemoteCmd struct {
	Throttle float32 // 油门 0~1
	Roll     float32 // 滚转 -1~+1 (左负右正)
	Pitch    float32 // 俯仰 -1~+1 (前负后正)
	Yaw      float32 // 偏航 -1~+1 (左负右正)
}

// Encode 编码为 19 字节帧 (含 XOR 校验)
func (c *RemoteCmd) Encode() []byte {
	buf := make([]byte, frameSize)
	buf[0] = frameHeader1
	buf[1] = frameHeader2
	binary.LittleEndian.PutUint32(buf[2:6], math.Float32bits(c.Throttle))
	binary.LittleEndian.PutUint32(buf[6:10], math.Float32bits(c.Roll))
	binary.LittleEndian.PutUint32(buf[10:14], math.Float32bits(c.Pitch))
	binary.LittleEndian.PutUint32(buf[14:18], math.Float32bits(c.Yaw))
	var crc byte
	for i := 0; i < frameSize-1; i++ {
		crc ^= buf[i]
	}
	buf[18] = crc
	return buf
}

func (c *RemoteCmd) String() string {
	return fmt.Sprintf("T=%.2f R=%.2f P=%.2f Y=%.2f", c.Throttle, c.Roll, c.Pitch, c.Yaw)
}

// RemoteController 遥控控制器
type RemoteController struct {
	port     serial.Port
	portName string
	baudRate int
	cmd      RemoteCmd
	running  bool
	txCount  uint64
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
	rc.txCount++
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
	portName := flag.String("port", "", "serial port (COMx / /dev/ttyUSB0); ESP-NOW 时接地面 ESP 的 USB 口")
	baudRate := flag.Int("baud", 115200, "serial baud rate")
	freq := flag.Int("freq", 50, "send frequency Hz")
	throttle := flag.Float64("t", 0, "initial throttle 0~1")
	roll := flag.Float64("r", 0, "initial roll -1~1")
	pitch := flag.Float64("p", 0, "initial pitch -1~1")
	yaw := flag.Float64("y", 0, "initial yaw -1~1")
	demo := flag.Bool("demo", false, "run demo mode (auto oscillate)")
	interactive := flag.Bool("i", true, "interactive stdin commands (t+/r-/z/q)")
	flag.Parse()

	if *portName == "" {
		fmt.Println("Usage: pacer-remote -port <device> [options]")
		fmt.Println("\nOptions:")
		flag.PrintDefaults()
		fmt.Println("\nExamples:")
		fmt.Println("  直连飞控 USART3:  pacer-remote -port COM5")
		fmt.Println("  ESP-NOW 地面端:   pacer-remote -port COM6 -demo")
		fmt.Println("  固定油门测试:     pacer-remote -port COM6 -t 0 -i=false")
		os.Exit(1)
	}

	rc, err := NewRemoteController(*portName, *baudRate)
	if err != nil {
		log.Fatalf("init remote controller: %v", err)
	}
	defer rc.Close()

	rc.SetThrottle(float32(*throttle))
	rc.SetRoll(float32(*roll))
	rc.SetPitch(float32(*pitch))
	rc.SetYaw(float32(*yaw))

	log.Printf("Remote controller started: %s @ %d bps, %d Hz", *portName, *baudRate, *freq)
	log.Printf("Initial: %s", rc.cmd.String())

	go rc.RunLoop(*freq)

	if *demo {
		go runDemo(rc)
	}

	if *interactive {
		runInteractive(rc)
	} else {
		waitForSignal(rc)
	}
}

func waitForSignal(rc *RemoteController) {
	sigCh := make(chan os.Signal, 1)
	signal.Notify(sigCh, syscall.SIGINT, syscall.SIGTERM)
	<-sigCh
	shutdown(rc)
}

func shutdown(rc *RemoteController) {
	fmt.Println("\nShutting down, sending zero frame...")
	rc.Reset()
	_ = rc.SendCmd()
	rc.Close()
}

// runDemo 演示模式 - 自动摇摆
func runDemo(rc *RemoteController) {
	t := 0.0
	for rc.running {
		time.Sleep(50 * time.Millisecond)
		t += 0.05
		rc.SetThrottle(0.0)
		rc.SetRoll(float32(0.3 * math.Sin(t)))
		rc.SetPitch(float32(0.2 * math.Cos(t*0.7)))
		rc.SetYaw(float32(0.1 * math.Sin(t * 0.5)))
	}
}

// runInteractive 交互式命令行控制
func runInteractive(rc *RemoteController) {
	fmt.Println()
	fmt.Println("=== Interactive Control ===")
	fmt.Println("  t+/t-  throttle ±0.05    T <value>  set throttle")
	fmt.Println("  r+/r-  roll ±0.1         R <value>  set roll")
	fmt.Println("  p+/p-  pitch ±0.1        P <value>  set pitch")
	fmt.Println("  y+/y-  yaw ±0.1          Y <value>  set yaw")
	fmt.Println("  z      reset all         s          show current")
	fmt.Println("  q      quit")
	fmt.Println("============================")

	sigCh := make(chan os.Signal, 1)
	signal.Notify(sigCh, syscall.SIGINT, syscall.SIGTERM)

	go func() {
		<-sigCh
		shutdown(rc)
		os.Exit(0)
	}()

	go func() {
		ticker := time.NewTicker(3 * time.Second)
		defer ticker.Stop()
		for rc.running {
			<-ticker.C
			log.Printf("tx=%d  %s", rc.txCount, rc.cmd.String())
		}
	}()

	scanner := bufio.NewScanner(os.Stdin)
	for scanner.Scan() {
		if !rc.running {
			break
		}
		line := strings.TrimSpace(scanner.Text())
		if line == "" {
			continue
		}
		parts := strings.Fields(line)
		cmd := strings.ToLower(parts[0])

		switch cmd {
		case "q", "quit", "exit":
			shutdown(rc)
			os.Exit(0)
		case "z", "reset":
			rc.Reset()
			fmt.Println("reset to zero")
		case "s", "status":
			fmt.Println(rc.cmd.String(), "tx=", rc.txCount)
		case "t+":
			rc.SetThrottle(rc.cmd.Throttle + 0.05)
		case "t-":
			rc.SetThrottle(rc.cmd.Throttle - 0.05)
		case "r+":
			rc.SetRoll(rc.cmd.Roll + 0.1)
		case "r-":
			rc.SetRoll(rc.cmd.Roll - 0.1)
		case "p+":
			rc.SetPitch(rc.cmd.Pitch + 0.1)
		case "p-":
			rc.SetPitch(rc.cmd.Pitch - 0.1)
		case "y+":
			rc.SetYaw(rc.cmd.Yaw + 0.1)
		case "y-":
			rc.SetYaw(rc.cmd.Yaw - 0.1)
		case "t", "throttle":
			if len(parts) < 2 {
				fmt.Println("usage: T <0~1>")
				continue
			}
			var v float32
			fmt.Sscanf(parts[1], "%f", &v)
			rc.SetThrottle(v)
		case "r", "roll":
			if len(parts) < 2 {
				continue
			}
			var v float32
			fmt.Sscanf(parts[1], "%f", &v)
			rc.SetRoll(v)
		case "p", "pitch":
			if len(parts) < 2 {
				continue
			}
			var v float32
			fmt.Sscanf(parts[1], "%f", &v)
			rc.SetPitch(v)
		case "y", "yaw":
			if len(parts) < 2 {
				continue
			}
			var v float32
			fmt.Sscanf(parts[1], "%f", &v)
			rc.SetYaw(v)
		default:
			fmt.Println("unknown command:", line)
			continue
		}
		fmt.Println("->", rc.cmd.String())
	}
}
