import struct,os,sys,time
import usb.core
import usb.util

VID = 0x0403
PID = 0x601f

# Adjust if your FT601 enumerates differently.
EP_OUT_CTRL = 0x01   # the endpoint that matches Bo:...:1 in your usbmon
EP_OUT_CMD = 0x02   # bulk OUT
EP_IN_DATA = 0x82   # bulk IN

def send_ctrl(dev, words):
    pkt = struct.pack(">" + "I"*len(words), *words)
    n = dev.write(EP_OUT_CTRL, pkt, timeout=1000)
    #print("ctrl wrote", n, "bytes:", " ".join(f"{w:08x}" for w in words))
    return n

def words_le(data: bytes):
    assert len(data) % 4 == 0
    return list(struct.unpack(">" + "I" * (len(data) // 4), data))

def read_cmd_reg(dev, addr,length=7,verbose=0):
    # Matches usbmon pattern: 00000000 00061177 for addr=0x0006
    w0 = 0x00000000
    w1 = ((addr & 0xffff) << 16) | 0x1177
    #pkt = struct.pack("<II", w0, w1)
    # 6, c, e
    addr1=0xc
    addr2=0xe
    addr3=8
    addr4=10
    magic=0x1377

    d=[]
    d+=[0x66665555]*4
    #first=6+3*1
    first=9 #+3
    first=addr
    for i in list(range(first,first+length)):
        d+=[0,(((i*2) & 0xffff) << 16) | magic]
    
    pkt = struct.pack(">" + "I"*len(d),
                      *d
                      )

    n = dev.write(EP_OUT_CMD, pkt, timeout=1000)
    if verbose: print(f"wrote {n} bytes: "+ " ".join(["%08x"%x for x in struct.unpack("<"+"I"*len(d),pkt)]))

    rlen=((len(d)-4)//2)*4

    if 1: #while 1:
        data = bytes(dev.read(EP_IN_DATA, rlen+8*300, timeout=1000))
    #data2 = bytes(dev.read(EP_IN_DATA, rlen, timeout=1000))
    ws = words_le(data)
    if verbose: print("reply words(%d %d):"%(rlen//4,len(data)//4))
    for i, w in enumerate(ws):
        if verbose or i>=6:
            print(f"  [{i+first-6}] {w:08x}")

    # Expected format from your serializer:
    # words[0:5] = 0x66665555 resync
    # words[5:8] = 3 payload words from mux
    #
    # For CMD responses, the useful 16-bit value is in one of the trailing words.
    # From your logs, word[6] usually carries the address/flags and word[7] the value or padding.
    return ws

def main(argv, stdout, environ):
    progname = argv[0]
    args=argv[1:]

    dev = usb.core.find(idVendor=VID, idProduct=PID)
    if dev is None:
        raise RuntimeError("FT601 device not found")

    try:
        dev.set_configuration()
    except usb.core.USBError:
        pass

    first = int(args[0])
    step=7
    #step=20
    total=48
    for i in range(first,first+total,step):
        #print("cycle")
        if 0:
            send_ctrl(dev, [
                0x01000000,
                0x82010000,
                0x00000200,
                0x00000000,
                0x00000000,
            ])
            #time.sleep(0.2)
            
        if 1:
            send_ctrl(dev, [
                0x01000000,
                0x82010000,
                0x00100000,
                0x00000000,
                0x00000000,
            ])
            #time.sleep(0.2)
        #ws = read_cmd_reg(dev, 19)
        ws = read_cmd_reg(dev, i, length=step)

        #time.sleep(0.2)
    
    #ws = read_cmd_reg(dev, 0x0007)
    #ws = read_cmd_reg(dev, 0x0008)

    # Heuristic decode based on your observed 52-byte frames:
    # final payload words are usually ws[5], ws[6], ws[7]
    #tail = ws[-3:]
    #print("tail:", " ".join(f"{w:08x}" for w in tail))

if __name__ == "__main__":
  main(sys.argv, sys.stdout, os.environ)

