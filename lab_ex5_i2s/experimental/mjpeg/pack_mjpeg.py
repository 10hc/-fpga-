"""Pack 320x240 JPEGs into a sector-aligned, bounded hardware container.

This is an asset preparation tool, never a runtime coprocessor. MJP1 is a
project-specific container; it is NOT an AVI/MOV/MP4 parser. Existing VID1
files are not accepted by the new experimental source.
"""
from pathlib import Path
from PIL import Image
import argparse, io, struct, json

SECTOR=512
MAX_JPEG=131072
def pad(data):
    return data + bytes((-len(data)) % SECTOR)

def pack(paths, output, fps=20, quality=80):
    if not 1 <= fps <= 60: raise ValueError('fps must be 1..60')
    if not paths: raise ValueError('At least one image is required')
    frames=[]
    for path in paths:
        image=Image.open(path)
        if image.size != (320,240):
            raise ValueError(f'{path}: expected 320x240, got {image.size}')
        buffer=io.BytesIO()
        # Standardize baseline 4:2:0: predictable row-buffer use and lower cost.
        image.convert('RGB').save(buffer,format='JPEG',quality=quality,subsampling=2,progressive=False)
        data=buffer.getvalue()
        if not 0 < len(data) <= MAX_JPEG: raise ValueError(f'JPEG too large: {path}')
        frames.append(data)
    sectors=1+sum(1+(len(d)+511)//512 for d in frames)
    header=struct.pack('<4sHHHHII',b'MJP1',320,240,fps,1,len(frames),sectors)
    output=Path(output)
    if output.exists(): raise FileExistsError(f'Refusing to overwrite {output}')
    with output.open('xb') as f:
        f.write(header.ljust(SECTOR,b'\0'))
        for index,data in enumerate(frames):
            pts=index*48000//fps
            f.write(struct.pack('<4sIII',b'JPG1',len(data),index,pts).ljust(SECTOR,b'\0'))
            f.write(pad(data))
    return dict(format='MJP1',width=320,height=240,fps=fps,frames=len(frames),
                sectors=sectors,bytes=sectors*512,jpeg_bytes=[len(d) for d in frames])

def inspect(path):
    with Path(path).open('rb') as f:
        h=f.read(SECTOR)
        if len(h)!=SECTOR: raise ValueError('Short file header')
        magic,w,hgt,fps,version,count,sectors=struct.unpack('<4sHHHHII',h[:20])
        if (magic,w,hgt,version)!=(b'MJP1',320,240,1) or not 1<=fps<=60 or count==0:
            raise ValueError('Invalid file header')
        if Path(path).stat().st_size!=sectors*512: raise ValueError('Length mismatch')
        frames=[]
        for i in range(count):
            d=f.read(SECTOR)
            if len(d)!=SECTOR: raise ValueError('Short frame header')
            marker,length,index,pts=struct.unpack('<4sIII',d[:16])
            if marker!=b'JPG1' or index!=i or not 0<length<=MAX_JPEG or pts!=i*48000//fps:
                raise ValueError('Invalid frame descriptor')
            data=f.read((length+511)//512*512)
            if len(data)<length or not data.startswith(b'\xff\xd8') or data[length-2:length]!=b'\xff\xd9':
                raise ValueError('Invalid JPEG boundaries')
            Image.open(io.BytesIO(data[:length])).load()
            frames.append(dict(index=index,pts=pts,bytes=length))
        if f.tell()!=sectors*512: raise ValueError('Trailing or missing sectors')
    return dict(format='MJP1',width=w,height=hgt,fps=fps,frames=frames,sectors=sectors)

if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('images',nargs='*',type=Path)
    p.add_argument('--output',type=Path)
    p.add_argument('--fps',type=int,default=20)
    p.add_argument('--quality',type=int,default=80)
    p.add_argument('--inspect',type=Path)
    a=p.parse_args()
    if a.inspect: result=inspect(a.inspect)
    elif a.output: result=pack(a.images,a.output,a.fps,a.quality)
    else: p.error('--output or --inspect required')
    print(json.dumps(result,indent=2))
