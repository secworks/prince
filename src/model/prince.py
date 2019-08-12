
class Prince():
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

    NUM_ROUNDS = 10


    def __init__(self, key, debug = True):
        self.debug = debug
        assert (len(key)) == 16, "Key must be 16 bytes."
        self.k0 = key[:8]
        self.k1 = key[8:]
        self.state = [0] * 16

        if self.debug:
            print("Subkey k0:", self.k0)
            print("Subkey k0:", self.k1)


    def encrypt(self, block):
        self.state = block[:]
        self.state = self.__xor_block(self.state, self.k0)
        for i in(range(self.NUM_ROUNDS)):
            self.state = self.__xor_block(self.state, self.rc[i])
            self.state = self.__sbox_block(self.state)
        return self.state


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


#-------------------------------------------------------------------
#-------------------------------------------------------------------
def main():
    my_key = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
    my_block = [0, 0, 0, 0, 0, 0, 0, 0]
    print("Testing the Prince implementation.")
    debug = True
    my_cipher = Prince(my_key, debug)
    my_res = my_cipher.encrypt(my_block)
    print(my_res)


#-------------------------------------------------------------------
# __name__
# Python thingy which allows the file to be run standalone as
# well as parsed from within a Python interpreter.
#-------------------------------------------------------------------
if __name__=="__main__":
    # Run the main function.
    sys.exit(main())
