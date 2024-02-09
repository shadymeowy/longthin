#include "stdio.h"
#include "stdlib.h"
#include "string.h"
#include "chacha.h"

int main(int argc, char *argv[])
{
    struct chacha_t ctx;
    chacha_init(&ctx, (uint8_t *)"12345678901234567890123456789012");
    chacha_nonce_set32(&ctx, (uint32_t[]){0, 0});

    uint8_t data[] = "Hello, world!";
    chacha_crypt(&ctx, data, strlen((char *)data));
    for (int i = 0; i < strlen((char *)data); i++)
    {
        printf("%02x", data[i]);
    }
    printf("\n");

    chacha_crypt(&ctx, data, strlen((char *)data));
    for (int i = 0; i < strlen((char *)data); i++)
    {
        printf("%02x", data[i]);
    }
    printf("\n");
    printf("%s\n", data);

    FILE *file = fopen("test.txt", "r");
    if (file == NULL)
    {
        printf("Error opening file\n");
        return 1;
    }
    fseek(file, 0, SEEK_END);
    long file_size = ftell(file);
    fseek(file, 0, SEEK_SET);
    uint8_t *file_data = malloc(file_size);
    fread(file_data, 1, file_size, file);
    fclose(file);
    chacha_crypt(&ctx, file_data, file_size);

    FILE *file_out = fopen("test.enc", "w");
    if (file_out == NULL)
    {
        printf("Error opening file\n");
        return 1;
    }
    fwrite(file_data, 1, file_size, file_out);
    fclose(file_out);
    free(file_data);
    return 0;
}