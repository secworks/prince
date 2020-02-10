#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#=======================================================================
#
# prince.py
# ---------
# Simple, pure Python, model of the PRINCE block cipher.
#
#
# Author: Joachim Strömbergson
# Copyright (c) 2020, Secworks Sweden AB
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or
# without modification, are permitted provided that the following
# conditions are met:
#
# 1. Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#
# 2. Redistributions in binary form must reproduce the above copyright
#    notice, this list of conditions and the following disclaimer in
#    the documentation and/or other materials provided with the
#    distribution.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
# STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF
# ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
#
#=======================================================================

#-------------------------------------------------------------------
# Python module imports.
#-------------------------------------------------------------------
import sys


#-------------------------------------------------------------------
# PRINCE
#-------------------------------------------------------------------
class PRINCE():
    VERBOSE = True
    DUMP_VARS = True
    NUM_ROUNDS = 10

    sbox = [0xb, 0xf, 0x3, 0x2, 0xa, 0xc, 0x9, 0x1,
            0x6, 0x7, 0x8, 0x0, 0xe, 0x5, 0xd, 0x4]

    isbox = [0xb, 0x7, 0x3, 0x2, 0xf, 0xd, 0x8, 0x9,
             0xa, 0x6, 0x4, 0x0, 0x5, 0xe, 0xc, 0x1]

    rc = [[0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00],
          [0x13, 0x19, 0x8a, 0x2e, 0x03, 0x70, 0x73, 0x44],
          [0xa4, 0x09, 0x38, 0x22, 0x29, 0x9f, 0x31, 0xd0],
          [0x08, 0x2e, 0xfa, 0x98, 0xec, 0x4e, 0x6c, 0x89],
          [0x45, 0x28, 0x21, 0xe6, 0x38, 0xd0, 0x13, 0x77],
          [0xbe, 0x54, 0x66, 0xcf, 0x34, 0xe9, 0x0c, 0x6c],
          [0x7e, 0xf8, 0x4f, 0x78, 0xfd, 0x95, 0x5c, 0xb1],
          [0x85, 0x84, 0x08, 0x51, 0xf1, 0xac, 0x43, 0xaa],
          [0xc8, 0x82, 0xd3, 0x2f, 0x25, 0x32, 0x3c, 0x54],
          [0x64, 0xa5, 0x11, 0x95, 0xe0, 0xe3, 0x61, 0x0d],
          [0xd3, 0xb5, 0xa3, 0x99, 0xca, 0x0c, 0x23, 0x99],
          [0xc0, 0xac, 0x29, 0xb7, 0xc9, 0x7c, 0x50, 0xdd]]

    ALPHA = 0xc0ac29b7c97c50dd


    #-------------------------------------------------------------------
    # __init__()
    #-------------------------------------------------------------------
    def __init__(self, key, debug = True):
        self.key = key
        self.debug = debug
        if self.debug:
            print("Init:")
            print("key: 0x%032x" % self.key)
            print("")


    #-------------------------------------------------------------------
    # encrypt()
    #-------------------------------------------------------------------
    def encrypt(self, plaintext):
        # Calculate the keys
        self.k0 = self.key >> 64
        self.k1 = self.key & (2**64 - 1)
        self.kp = self.__ror64(self.k0, 1) ^ self.k0 >> 63

        self.core_input = plaintext ^ self.k0;

        self.r0 = self.__first(self.core_input)

        self.r1 = self.__round(self.r0, 1)
        self.r2 = self.__round(self.r1, 2)
        self.r3 = self.__round(self.r2, 3)
        self.r4 = self.__round(self.r3, 4)
        self.r5 = self.__round(self.r4, 5)

        self.middle = self.__middle(self.r5)

        self.r6  = self.__inv_round(self.middle, 6)
        self.r7  = self.__inv_round(self.r6, 7)
        self.r8  = self.__inv_round(self.r7, 8)
        self.r9  = self.__inv_round(self.r8, 9)
        self.r10 = self.__inv_round(self.r9, 10)

        self.core_output = self.__final(self.r10)

        self.ciphertext = self.core_output ^ self.kp


        if self.debug:
            print("Encrypt:")
            print("k0:          0x%016x" % self.k0)
            print("k1:          0x%016x" % self.k1)
            print("kp:          0x%016x" % self.kp)
            print("core_input:  0x%016x" % self.core_input)
            print("round 0:     0x%016x" % self.r0)
            print("round 1:     0x%016x" % self.r1)
            print("round 2:     0x%016x" % self.r2)
            print("round 3:     0x%016x" % self.r3)
            print("round 4:     0x%016x" % self.r4)
            print("round 5:     0x%016x" % self.r5)
            print("middle:      0x%016x" % self.middle)
            print("round 6:     0x%016x" % self.r6)
            print("round 7:     0x%016x" % self.r7)
            print("round 8:     0x%016x" % self.r8)
            print("round 9:     0x%016x" % self.r9)
            print("round 10:    0x%016x" % self.r10)
            print("core_output: 0x%016x" % self.core_output)
            print("ciphertext:  0x%016x" % self.ciphertext)
            print("")

        return self.ciphertext


    #-------------------------------------------------------------------
    # decrypt()
    #-------------------------------------------------------------------
    def decrypt(self, block):
        self.kp = self.key >> 64
        self.k1 = (self.key & (2**64 - 1)) ^ self.ALPHA
        self.k0 = self.__ror64(self.kp, 1) ^ self.kp >> 63
        self.core_input = block ^ self.k0;

        if self.debug:
            print("Decrypt:")
            print("k0:         0x%016x" % self.k0)
            print("k1:         0x%016x" % self.k1)
            print("kp:         0x%016x" % self.kp)
            print("core_input: 0x%016x" % self.core_input)
            print("")

        return self.core_input


    #-------------------------------------------------------------------
    # Internal methods.
    #-------------------------------------------------------------------
    def __first(self, b):
        return b ^ self.k1


    def __round(self, b, n):
        return b


    def __middle(self, b):
        return b


    def __inv_round(self, b, n):
        return b


    def __final(self, b):
        return b


    def __xor_block(self, b, d):
        assert len(b) == len(d), "Blocks must be of equal length"
        res = [0] * len(b)
        for i in range(len(b)):
            res[i] = b[i] ^ d[i]
        return res


    def __sbox_block(self, b):
        res = [0] * len(b)
        for i in range(len(b)):
            hn = b[i] >> 4
            ln = b[i] & 0xf
            res[i] = self.sbox[hn] * 16 + self.sbox[ln]
        return res


    def __isbox_block(self, b):
        res = [0] * len(b)
        for i in range(len(b)):
            hn = b[i] >> 4
            ln = b[i] & 0xf
            res[i] = self.isbox[hn] * 16 + self.isbox[ln]
        return res


    def __ror64(self, x, n):
        return ((x >> n) | (x << (64 - n))) & (2**64 - 1)


#-------------------------------------------------------------------
# Testing the cipher using test vectors from
# https://github.com/sebastien-riou/prince-c-ref/blob/master/log.txt
#-------------------------------------------------------------------
def test_cipher():
    tc1 = (0x00000000_00000000_00000000_00000000,
           0x00000000_00000000, 0x818665aa_0d02dfda)

    tc2 = (0x00000000_00000000_00000000_00000000,
           0xffffffff_ffffffff, 0x604ae6ca_03c20ada)

    tc3 = (0xffffffff_ffffffff_00000000_00000000,
           0x00000000_00000000, 0x9fb51935_fc3df524)

    tc4 = (0x00000000_00000000_ffffffff_ffffffff,
           0x00000000_00000000, 0x78a54cbe_737bb7ef)

    tc5 = (0x00000000_00000000_fedcba98_76543210,
           0x01234567_89abcdef, 0xae25ad3c_a8fa9ccf)

    tc6 = (0x00112233_44556677_8899aabb_ccddeeff,
           0x01234567_89abcdef, 0xd6dcb597_8de756ee)

    tc7 = (0x01122334_45566778_899aabbc_cddeeff0,
           0x01234567_89abcdef, 0x392f599f_46761cd3)

    tc8 = (0x01122334_45566778_899aabbc_cddeeff0,
           0xf0123456_789abcde, 0x4fb5e332_b9b409bb)

    tc9 = (0xd8cdb780_70b4c55a_818665aa_0d02dfda,
           0x69c4e0d8_6a7b0430, 0x43c6b256_d79de7e8)

    tests = [tc1, tc2, tc3, tc4, tc5, tc6, tc7, tc8, tc9]


    print("Testing the cipher.")

    for i in range(len(tests)):
        print("Testcase %d:" % (i + 1))
        (key, plaintext, ciphertext) = tests[i]
        my_cipher = PRINCE(key)
        c = my_cipher.encrypt(plaintext)
        p = my_cipher.decrypt(ciphertext)

        if c == ciphertext:
            print("Correct ciphertext generated.")
        else:
            print("Incorrect ciphertext generated.")

        if p == plaintext:
            print("Correct plaintext generated.")
        else:
            print("Incorrect plaintext generated.")

        print("")


#-------------------------------------------------------------------
#-------------------------------------------------------------------
def main():
    test_cipher()


#-------------------------------------------------------------------
# __name__
# Python thingy which allows the file to be run standalone as
# well as parsed from within a Python interpreter.
#-------------------------------------------------------------------
if __name__=="__main__":
    # Run the main function.
    sys.exit(main())

#=======================================================================
# EOF prince.py
#=======================================================================
