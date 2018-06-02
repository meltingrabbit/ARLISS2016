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

INPUT_FILE_NAME = 'LogFloat.csv'
OOUTPUT_FILE_NAME = 'LogGps.csv'



def main():
    outputFile = open(OOUTPUT_FILE_NAME, "w")
    for line in open(INPUT_FILE_NAME):
        line = line.strip()
        lines = line.split(',')
        if (len(lines) < 3):
            continue
        if not(lines[1] == 'Gps'):
            continue
        flag = 0
        for l in lines:
            if (flag == 0):
                flag = 1
            else:
                outputFile.write(',')
            outputFile.write(l)
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

