//--------------------------------------------------------------------------------------
//			RMPC Functions (13/04/23)
//--------------------------------------------------------------------------------------

// DBL_INT_ADD treats two unsigned ints a and b as one 64-bit integer and adds c to it
#define DBL_INT_ADD(a,b,c) if (a > 0xffffffff - (c)) ++b; a += c;
#define ROTLEFT(a,b) (((a) << (b)) | ((a) >> (32-(b))))
#define ROTRIGHT(a,b) (((a) >> (b)) | ((a) << (32-(b))))
#define CH(x,y,z) (((x) & (y)) ^ (~(x) & (z)))
#define MAJ(x,y,z) (((x) & (y)) ^ ((x) & (z)) ^ ((y) & (z)))
#define EP0(x) (ROTRIGHT(x,2) ^ ROTRIGHT(x,13) ^ ROTRIGHT(x,22))
#define EP1(x) (ROTRIGHT(x,6) ^ ROTRIGHT(x,11) ^ ROTRIGHT(x,25))
#define SIG0(x) (ROTRIGHT(x,7) ^ ROTRIGHT(x,18) ^ ((x) >> 3))
#define SIG1(x) (ROTRIGHT(x,17) ^ ROTRIGHT(x,19) ^ ((x) >> 10))
#define IPAD 0x36
#define OPAD 0x5C
   
typedef struct {
	unsigned char data[64];
	unsigned long datalen;
	unsigned long bitlen[2];
	unsigned long state[8];
} SHA256_CTX;

//SIC Functions
void sha256(unsigned char *text1 ,unsigned char *output,unsigned int length);
void sha256_init(SHA256_CTX *ctx);
void sha256_update(SHA256_CTX *ctx, unsigned char data[], unsigned long len);
void sha256_final(SHA256_CTX *ctx, unsigned char hash[]);
void sha256_transform(SHA256_CTX *ctx, unsigned char data[]);
unsigned int hmacsha256(unsigned char *key, unsigned int  key_len, unsigned char *text, unsigned int  text_len, unsigned char *digest);
unsigned int RPMC_UpHMACkey(struct spi_device *spi, unsigned int cadr,unsigned char *rootkey,unsigned char *hmac4,unsigned char *hmackey);
unsigned char RPMC_Challenge(struct spi_device *spi, unsigned int cadr, unsigned char *hmackey,unsigned char *input_tag);
unsigned int RPMC_IncCounter(struct spi_device *spi, unsigned int cadr,unsigned char *hmackey,unsigned char *input_tag);

