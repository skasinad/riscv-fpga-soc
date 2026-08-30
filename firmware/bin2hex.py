import sys

filename=sys.argv[1]

with open(filename,"rb") as f:
    data=f.read()

while len(data)%4:
    data+=b"\x00"

for i in range(0,len(data),4):
    word=int.from_bytes(data[i:i+4],"little")
    print(f"{word:08x}")