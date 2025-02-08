tcprewrite --infile=$1 --outfile=middle.pcap --dstipmap=0.0.0.0/0:192.168.86.50 --enet-dmac=98:90:96:aa:2e:0b; tcprewrite --infile=middle.pcap --outfile="remaped-$1" --fixcsum ; rm middle.pcap

