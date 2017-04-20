/**
  ******************************************************************************
  * @file    	/Rootkey.c
  * @brief   	This code provides sample to create the 256-bit rootkey by Unique ID & 256-bit Tag.
  *		User can modiy the file to create his rootkey. 
  *            
*/ 
unsigned int hmacsha256(unsigned char *key, unsigned int  key_len, unsigned char *text, unsigned int  text_len, unsigned char *digest);

unsigned char ROOTKeyTag[32];	// Rootkey array

/* rootkey information can up to 32byte (256bit) */
void RPMC_CreateRootKey(unsigned char *u8uid, unsigned int id_len, unsigned char *rootkey) 
{
	int i = 0;
 
	for(i=0;i<32;i++)
		ROOTKeyTag[i] = 0;

	ROOTKeyTag[0] = 'N';
	ROOTKeyTag[1] = 'u';
	ROOTKeyTag[2] = 'v';
	ROOTKeyTag[3] = 'o';
	ROOTKeyTag[4] = 't';
	ROOTKeyTag[5] = 'o';
	ROOTKeyTag[6] = 'n';

	hmacsha256((unsigned char *)u8uid, id_len, ROOTKeyTag,32,rootkey);
	
}
