# coding: UTF-8

"""
文字コードはutf-8N
改行コードは CRCF
Windows cmdではユニコード文字列を使うこと．
print u"にほんご"
print "にほんご"

2016/06/28

"""

import math
import sys

DEBUG = 0

INPUT_FILE_NAME = 'LogGpsPlotMeter.dat'
OOUTPUT_FILE_NAME = 'LogGpsLabelMeter.dat'



def main():
    outputFile = open(OOUTPUT_FILE_NAME, "w")
    count = 0
    for line in open(INPUT_FILE_NAME):
        count += 1
        line = line.strip()
        lines = line.split('\t')
        if (len(lines) < 3):
            continue
        time = float(lines[0])
        time = int(time)
        x = float(lines[2])
        y = float(lines[3])

        outputFile.write('set label ' + str(count) + ' at ' + '%.6f' % x + ',' + '%.6f' % y + ' \"' + str(time) + '\"')
        outputFile.write('\n')

    outputFile.close()




def IsNumeric(n):
    if (isinstance(n, int) or isinstance(n, float)):
        return True
    else:
        return False


def ps(input):
    sys.stdout.write(str(input))
    sys.stdout.flush()

# ユニコード文字列はstr()にいれるとバグったので
def pu(input):
    sys.stdout.write(input)
    sys.stdout.flush()

def pr(string):
    print (string)
    sys.stdout.flush()

if __name__ == '__main__':
    main()

