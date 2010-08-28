/* Daniel Spinner
   CSI445-Robotics
   Final Project - pocketsphinx_continuous mod
*/

/* -*- c-basic-offset: 4; indent-tabs-mode: nil -*- */
/* ====================================================================
 * Copyright (c) 1999-2001 Carnegie Mellon University.  All rights
 * reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer. 
 *
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 *
 * This work was supported in part by funding from the Defense Advanced 
 * Research Projects Agency and the National Science Foundation of the 
 * United States of America, and the CMU Sphinx Speech Consortium.
 *
 * THIS SOFTWARE IS PROVIDED BY CARNEGIE MELLON UNIVERSITY ``AS IS'' AND 
 * ANY EXPRESSED OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, 
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED.  IN NO EVENT SHALL CARNEGIE MELLON UNIVERSITY
 * NOR ITS EMPLOYEES BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT 
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, 
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY 
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT 
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE 
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 * ====================================================================
 *
 */
/*
 * demo.c -- An example SphinxII program using continuous listening/silence filtering
 * 		to segment speech into utterances that are then decoded.
 * 
 * HISTORY
 *
 * 15-Jun-99    Kevin A. Lenzo (lenzo@cs.cmu.edu) at Carnegie Mellon University
 *              Added i386_linux and used ad_open_sps instead of ad_open
 * 
 * 14-Jun-96	M K Ravishankar (rkm@cs.cmu.edu) at Carnegie Mellon University.
 * 		Created.
 */

/*
 * This is a simple, tty-based example of a SphinxII client that uses continuous listening
 * with silence filtering to automatically segment a continuous stream of audio input
 * into utterances that are then decoded.
 * 
 * Remarks:
 *   - Each utterance is ended when a silence segment of at least 1 sec is recognized.
 *   - Single-threaded implementation for portability.
 *   - Uses fbs8 audio library; can be replaced with an equivalent custom library.
 */

#include <stdio.h>
#include <string.h>

#include <stdlib.h>
#include <ctype.h>
#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/socket.h>

#include "pocketsphinx.h"
#include "err.h"
#include "ad.h"
#include "cont_ad.h"

#define ROBOT_IP "192.168.1.111"
#define ROBOT_PORT 10001
#define SIZE sizeof(struct sockaddr_in)

#if !defined(_WIN32_WCE)
#include <signal.h>
#include <setjmp.h>
#endif
#if defined(WIN32) && !defined(GNUWINCE)
#include <time.h>
#else
#include <sys/types.h>
#include <sys/time.h>
#endif

static const arg_t cont_args_def[] = {
    POCKETSPHINX_OPTIONS,
    /* Argument file. */
    { "-argfile",
      ARG_STRING,
      NULL,
      "Argument file giving extra arguments." },
    { "-adcdev", ARG_STRING, NULL, "Name of audio device to use for input." },
    CMDLN_EMPTY_OPTION
};

static ad_rec_t *ad;
static ps_decoder_t *ps;

/* Sleep for specified msec */
static void
sleep_msec(int32 ms)
{
#if (defined(WIN32) && !defined(GNUWINCE)) || defined(_WIN32_WCE)
    Sleep(ms);
#else
    /* ------------------- Unix ------------------ */
    struct timeval tmo;

    tmo.tv_sec = 0;
    tmo.tv_usec = ms * 1000;

    select(0, NULL, NULL, NULL, &tmo);
#endif
}

/*
 * Main utterance processing loop:
 *     for (;;) {
 * 	   wait for start of next utterance;
 * 	   decode utterance until silence of at least 1 sec observed;
 * 	   print utterance result;
 *     }
 */
static void

/***************************NETWORKING CODE NEEDS TO GO IN THIS FUNCTION*****************************/
utterance_loop()
{
    int16 adbuf[4096];
    int32 k, ts, rem, score;
    char const *hyp;
    char const *uttid;
    cont_ad_t *cont;
    char word[256];
    
    /***Networking Variables***/
     int sockfd;
     struct sockaddr_in targetBot;
	memset(&targetBot,0,sizeof(targetBot));
	targetBot.sin_family = AF_INET;
	targetBot.sin_port = htons(ROBOT_PORT);
	const char *PRE = "!msg ";
	const char *POST = "\n";
	char *toSend;
	char c;

    /* Initialize continuous listening module */
    if ((cont = cont_ad_init(ad, ad_read)) == NULL)
        E_FATAL("cont_ad_init failed\n");
    if (ad_start_rec(ad) < 0)
        E_FATAL("ad_start_rec failed\n");
    if (cont_ad_calib(cont) < 0)
        E_FATAL("cont_ad_calib failed\n");
        
     /* Set IP address of robot */
     targetBot.sin_addr.s_addr = inet_addr(ROBOT_IP);
	
	/* Initialize socket */
	if((sockfd = socket(AF_INET, SOCK_STREAM, 0)) == -1)
	{
		fprintf(stderr, "Socket call failed.\n");
		exit(1);
	}
	
	/* Connect to robot through socket */
	if(connect(sockfd, (struct sockaddr *)&targetBot, SIZE) == -1)
	{
		fprintf(stderr, "Connect call failed.\n");
		exit(1);
	}

    for (;;) {
        /* Indicate listening for next utterance */
        printf("READY....\n");
        fflush(stdout);
        fflush(stderr);

        /* Await data for next utterance */
        while ((k = cont_ad_read(cont, adbuf, 4096)) == 0)
            sleep_msec(200);

        if (k < 0)
            E_FATAL("cont_ad_read failed\n");

        /*
         * Non-zero amount of data received; start recognition of new utterance.
         * NULL argument to uttproc_begin_utt => automatic generation of utterance-id.
         */
        if (ps_start_utt(ps, NULL) < 0)
            E_FATAL("ps_start_utt() failed\n");
        ps_process_raw(ps, adbuf, k, FALSE, FALSE);
        printf("Listening...\n");
        fflush(stdout);

        /* Note timestamp for this first block of data */
        ts = cont->read_ts;

        /* Decode utterance until end (marked by a "long" silence, >1sec) */
        for (;;) {
            /* Read non-silence audio data, if any, from continuous listening module */
            if ((k = cont_ad_read(cont, adbuf, 4096)) < 0)
                E_FATAL("cont_ad_read failed\n");
            if (k == 0) {
                /*
                 * No speech data available; check current timestamp with most recent
                 * speech to see if more than 1 sec elapsed.  If so, end of utterance.
                 */
                if ((cont->read_ts - ts) > DEFAULT_SAMPLES_PER_SEC)
                    break;
            }
            else {
                /* New speech data received; note current timestamp */
                ts = cont->read_ts;
            }

            /*
             * Decode whatever data was read above.
             */
            rem = ps_process_raw(ps, adbuf, k, FALSE, FALSE);

            /* If no work to be done, sleep a bit */
            if ((rem == 0) && (k == 0))
                sleep_msec(20);
        }

        /*
         * Utterance ended; flush any accumulated, unprocessed A/D data and stop
         * listening until current utterance completely decoded
         */
        ad_stop_rec(ad);
        while (ad_read(ad, adbuf, 4096) >= 0);
        cont_ad_reset(cont);

        printf("Stopped listening, please wait...\n");
        fflush(stdout);
        /* Finish decoding, obtain and print result */
        ps_end_utt(ps);
/********************HERE IS WHERE THE OUTPUT HYPOTHESIS IS**********************************/
        hyp = ps_get_hyp(ps, &score, &uttid);
        printf("%s: %s (%d)\n", uttid, hyp, score);
        fflush(stdout);
        
        
/**************Network Transmission Code****************/
        /* End of voice decoding, start of text message transmission. */
	
	/* Allocate memory for string to be sent. */
	if((toSend = (char *)calloc(strlen(PRE) + strlen(hyp) + 2, sizeof(char))) == NULL)
	{
		fprintf(stderr, "Calloc failed.\n");
		exit(1);
	}
	
	/* Add necessary prefix and suffix to message */
	strcat(toSend, PRE);
	strcat(toSend, hyp);
	strcat(toSend, POST);
	
	/* Send message to robot. */
	if((send(sockfd, toSend, strlen(toSend), 0)) == -1)
		printf("Error sending.\n");
	else
		printf("Sending...\n");
	
	/* Read in reply from robot. */
/*	while(read(sockfd,&c,1))
		printf("%c",c);
*/	
	
        
        

        /* Exit if the word spoken was "THATS ALL".  Change string in strcmp() for another phrase. */
        if (hyp) {
            sscanf(hyp, "%s", word);
            if (strcmp(hyp, "THATS ALL") == 0)
                break;
        }

        /* Resume A/D recording for next utterance */
        if (ad_start_rec(ad) < 0)
            E_FATAL("ad_start_rec failed\n");
    }

    close(sockfd);
    free(toSend);
    cont_ad_close(cont);
}

static jmp_buf jbuf;
static void
sighandler(int signo)
{
    longjmp(jbuf, 1);
}

int
main(int argc, char *argv[])
{
    cmd_ln_t *config;
    char const *cfg;

    /* Make sure we exit cleanly (needed for profiling among other things) */
    /* Signals seem to be broken in arm-wince-pe. */
#if !defined(GNUWINCE) && !defined(_WIN32_WCE)
    signal(SIGINT, &sighandler);
#endif

    if (argc == 2) {
        config = cmd_ln_parse_file_r(NULL, cont_args_def, argv[1], TRUE);
    }
    else {
        config = cmd_ln_parse_r(NULL, cont_args_def, argc, argv, TRUE);
    }
    /* Handle argument file as -argfile. */
    if (config && (cfg = cmd_ln_str_r(config, "-argfile")) != NULL) {
        config = cmd_ln_parse_file_r(config, cont_args_def, cfg, FALSE);
    }
    if (config == NULL)
        return 1;
    ps = ps_init(config);
    if (ps == NULL)
        return 1;

    if ((ad = ad_open_dev(cmd_ln_str_r(config, "-adcdev"),
                          (int)cmd_ln_float32_r(config, "-samprate"))) == NULL)
        E_FATAL("ad_open_dev failed\n");

    E_INFO("%s COMPILED ON: %s, AT: %s\n\n", argv[0], __DATE__, __TIME__);

    if (setjmp(jbuf) == 0) {
        utterance_loop();
    }

    ps_free(ps);
    ad_close(ad);

    return 0;
}
