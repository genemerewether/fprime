#!/usr/bin/env python

import os
import datetime
import struct
import argparse

PAD_ALIGN = 4096

def main():
    parser = argparse.ArgumentParser(description='Unpack buffer logger file',
                                     formatter_class=argparse.ArgumentDefaultsHelpFormatter)
    parser.add_argument('in_files', type=str, help='Path to input file(s)', nargs='+')
    parser.add_argument('-s', '--start_idx', type=int, default=0,
                        help='Start index for individual files')
    parser.add_argument('-w', '--warn_size', type=int, default=20*1000*1000,
                        help='Buffer size to warn if greater than')
    parser.add_argument('-r', '--regular_write', action="store_true", default=False,
                        help='Set if BufferLogger was not used with BL_DIRECT_WRITE')
    parser.add_argument('-o', '--output', action='store', type=str,
                        default='./out_'+datetime.datetime.now().strftime('%Y_%m_%d-%H_%M_%S'),
                        help='Output folder')

    args = parser.parse_args()

    seq = args.start_idx - 1
    if not os.path.isdir(args.output):
        try:
            os.mkdir(args.output)
        except Exception as e:
            print(e)
            print('Failed to create output directory')
            exit(1)

    for in_file in args.in_files:
        file_size = os.path.getsize(in_file)
        read_so_far = 0

        with open(in_file, 'rb') as filp:
            basename = os.path.split(in_file)[1]

            fmt = '>I'
            while (read_so_far + struct.calcsize(fmt) < file_size):
                numBytes = struct.unpack(fmt, filp.read(struct.calcsize(fmt)))[0]
                if numBytes > args.warn_size:
                    raw_input("Buffer may have lost sync - check your " +
                              "Svc::BufferLogger settings and maybe retry " +
                              "with/out -r option.\nTo silence this warning," +
                              "use the -w flag with the min size to complain " +
                              "about.\n Read size %d"%(numBytes))
                if not args.regular_write:
                    # discard empty bytes, align to 512
                    filp.read(PAD_ALIGN - struct.calcsize(fmt))
                    read_so_far += PAD_ALIGN
                else:
                    read_so_far += struct.calcsize(fmt)

                if (read_so_far + numBytes > file_size):
                    print("Read hit EOF on file %s, %d bytes left but need %d"%(
                          in_file, file_size - read_so_far, numBytes))
                    continue

                in_buf = filp.read(numBytes)
                read_so_far += numBytes
                seq += 1
                with open(os.path.join(args.output, basename + "_%05d"%seq), 'wb') as outFilp:
                    outFilp.write(in_buf)

                # NOTE(mereweth) - NavCam and RteCam use width-aligned buffers
                if not args.regular_write and numBytes % PAD_ALIGN != 0:
                    # discard empty bytes, align to 512
                    alignedBufSize = (int(numBytes / PAD_ALIGN) + 1) * PAD_ALIGN
                    filp.read(alignedBufSize - numBytes)
                    read_so_far += (alignedBufSize - numBytes)

if __name__ == '__main__':
    main()
