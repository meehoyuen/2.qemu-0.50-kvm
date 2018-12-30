/*
 * create a COW disk image
 */
#include <stdlib.h>
#include <stdio.h>
#include <stdarg.h>
#include <string.h>
#include <getopt.h>
#include <inttypes.h>
#include <unistd.h>
#include <sys/mman.h>
#include <fcntl.h>
#include <signal.h>
#include <time.h>
#include <sys/time.h>
#include <malloc.h>
#include <termios.h>
#include <sys/poll.h>
#include <errno.h>
#include <sys/wait.h>
#include <sys/stat.h>
#include <netinet/in.h>

#include "vl.h"

#include "bswap.h"

int cow_create(int cow_fd, const char *image_filename, 
               int64_t image_sectors)
{
    struct cow_header_v2 cow_header;
    int fd;
    struct stat st;

    memset(&cow_header, 0, sizeof(cow_header));
    cow_header.magic = htonl(COW_MAGIC);
    cow_header.version = htonl(COW_VERSION);
    if (image_filename) {
        fd = open(image_filename, O_RDONLY);
        if (fd < 0) {
            perror(image_filename);
            exit(1);
        }
        image_sectors = lseek64(fd, 0, SEEK_END);
        if (fstat(fd, &st) != 0) {
            close(fd);
            return -1;
        }
        close(fd);
        image_sectors /= 512;
        cow_header.mtime = htonl(st.st_mtime);
        realpath(image_filename, cow_header.backing_file);
    }
    cow_header.sectorsize = htonl(512);
    cow_header.size = image_sectors * 512;
#ifndef WORDS_BIGENDIAN
    cow_header.size = bswap64(cow_header.size);
#endif    
    write(cow_fd, &cow_header, sizeof(cow_header));
    /* resize to include at least all the bitmap */
    ftruncate(cow_fd, sizeof(cow_header) + ((image_sectors + 7) >> 3));
    lseek(cow_fd, 0, SEEK_SET);
    return 0;
}

void help(void)
{
    printf("vlmkcow version " QEMU_VERSION ", Copyright (c) 2003 Fabrice Bellard\n"
           "usage: vlmkcow [-h] [-f disk_image] cow_image [cow_size]\n"
           "Create a Copy On Write disk image from an optional raw disk image\n"
           "\n"
           "-f disk_image   set the raw disk image name\n"
           "cow_image       the created cow_image\n"
           "cow_size        the create cow_image size in MB if no raw disk image is used\n"
           "\n"
           "Once the cow_image is created from a raw disk image, you must not modify the original raw disk image\n"
           );
    exit(1);
}

int main(int argc, char **argv)
{
    const char *image_filename, *cow_filename;
    int cow_fd, c, nb_args;
    int64_t image_size;
    
    image_filename = NULL;
    image_size = 0;
    for(;;) {
        c = getopt(argc, argv, "hf:");
        if (c == -1)
            break;
        switch(c) {
        case 'h':
            help();
            break;
        case 'f':
            image_filename = optarg;
            break;
        }
    }
    if (!image_filename)
        nb_args = 2;
    else
        nb_args = 1;
    if (optind + nb_args != argc)
        help();

    cow_filename = argv[optind];
    if (nb_args == 2) {
        image_size = (int64_t)atoi(argv[optind + 1]) * 2 * 1024;
    }

    cow_fd = open(cow_filename, O_RDWR | O_CREAT | O_TRUNC, 0644);
    if (!cow_fd < 0)
        return -1;
    if (cow_create(cow_fd, image_filename, image_size) < 0) {
        fprintf(stderr, "%s: error while formating\n", cow_filename);
        exit(1);
    }
    close(cow_fd);
    return 0;
}
