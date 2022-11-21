// ShmReader.cpp : Sample implementation of a process reading
// from a shared memory segment (double buffered) with RDB layout
// (c) 2016 by VIRES Simulationstechnologie GmbH
// Provided AS IS without any warranty!
//

// Tianshu 2021.2.7 save image to file *.rgb

#include <stdlib.h>
#include <stdio.h>
#include <sys/shm.h>
#include <string.h>
#include <unistd.h>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <cstring>
#include <sstream>
#include "RDBHandler.hh"
#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

// forward declarations of methods

/**
* method for checking the contents of the SHM
*/
int  checkShm();
void openShm();

/**
* routine for handling an RDB message; to be provided by user;
* here, only a printing of the message is performed
* @param msg    pointer to the message that is to be handled
*/
void handleMessage( RDB_MSG_t* msg );

/**
 * Parse message and print it out
 * @param simTime
 * @param simFrame
 * @param entryHdr
 */
void parseRDBMessageEntry( const double & simTime, const unsigned int & simFrame, RDB_MSG_ENTRY_HDR_t* entryHdr, int& counter);

/**
 * Handle a RDBImage and print it out
 * @param simTime
 * @param simFrame
 * @param img
 */
void handleRDBitem(const double & simTime, const unsigned int & simFrame, RDB_IMAGE_t* img, int counter);

/**
* some global variables, considered "members" of this example
*/
// unsigned int mShmKey       = 0x8201;                            // key of the SHM segment
// unsigned int mShmKey       = 0x816a;                            // key of the SHM segment
unsigned int mShmKey       = 0x08130;                            // key of the SHM segment
unsigned int mCheckMask    = RDB_SHM_BUFFER_FLAG_TC;
void*        mShmPtr       = 0;                                 // pointer to the SHM segment
size_t       mShmTotalSize = 0;                                 // remember the total size of the SHM segment
bool         mVerbose      = false;                             // run in verbose mode?
int          mForceBuffer  = -1;                                // force reading one of the SHM buffers (0=A, 1=B)

/**
* information about usage of the software
* this method will exit the program
*/
void usage()
{
    printf("usage: shmReader [-k:key] [-c:checkMask] [-v] [-f:bufferId]\n\n");
    printf("       -k:key        SHM key that is to be addressed\n");
    printf("       -c:checkMask  mask against which to check before reading an SHM buffer\n");
    printf("       -f:bufferId   force reading of a given buffer (0 or 1) instead of checking for a valid checkMask\n");
    printf("       -v            run in verbose mode\n");
    exit(1);
}

enum HitInteractionType
{
    NONE,           // initial type     - shooting from sensor
    REFRACTION,     // refraction ray   - goes inside of a surface
    REFLECTION      // reflection ray   - bounces of a surface
};

/**
* validate the arguments given in the command line
*/
void ValidateArgs(int argc, char **argv)
{
    for( int i = 1; i < argc; i++)
    {
        if ((argv[i][0] == '-') || (argv[i][0] == '/'))
        {
            fprintf( stderr, "Reading parameters...\n" );
            switch (tolower(argv[i][1]))
            {
                case 'k':        // shared memory key
                    if ( strlen( argv[i] ) > 3 )
                    {
                        fprintf( stderr, "Reading key parameter...\n" );
                        mShmKey = atoi( &argv[i][3] );
                        fprintf( stderr, "found: %i\n", mShmKey );
                    }
                    break;
                    
                case 'c':       // check mask
                    if ( strlen( argv[i] ) > 3 )
                        mCheckMask = atoi( &argv[i][3] );
                    break;
                    
                case 'f':       // force reading a given buffer
                    if ( strlen( argv[i] ) > 3 )
                        mForceBuffer = atoi( &argv[i][3] );
                    break;
                    
                case 'v':       // verbose mode
                    mVerbose = true;
                    break;
                    
                default:
                    usage();
                    break;
            }
        }
    }
    
    fprintf( stderr, "ValidateArgs: key = 0x%x, checkMask = 0x%x, mForceBuffer = %d\n", 
                     mShmKey, mCheckMask, mForceBuffer );
}

/**
* main program with high frequency loop for checking the shared memory;
* does nothing else
*/

int main(int argc, char* argv[])
{
    // Parse the command line
    //
    ValidateArgs(argc, argv);
    
    // first: open the shared memory (try to attach without creating a new segment)
    
    fprintf( stderr, "attaching to shared memory....\n" );
    
    while ( !mShmPtr )
    {
        openShm();
        usleep( 100000 );     // do not overload the CPU
    }
    
    fprintf( stderr, "...attached! Reading now...\n" );
    
    // now check the SHM for the time being
    while ( 1 )
    {
        checkShm();
        
        usleep( 1000 );
    }
}

/**
* open the shared memory segment
*/
void openShm()
{
    // do not open twice!
    if ( mShmPtr )
    {
        fprintf( stderr, "openShm was already called\n" );
        return;
    }

    int shmid = 0; 

    if ( ( shmid = shmget( mShmKey, 0, 0 ) ) < 0 )
    {
        fprintf( stderr, "No matching key %i != %ui \n", shmid, mShmKey);
        return;
    }
    else
    {
        fprintf( stderr, "Matching key %i\n", shmid);
    }

    if ( ( mShmPtr = (char *)shmat( shmid, (char *)0, 0 ) ) == (char *) -1 )
    {
        perror("openShm: shmat()");
        mShmPtr = 0;
    }

    if ( mShmPtr )
    {
        struct shmid_ds sInfo;

        if ( ( shmid = shmctl( shmid, IPC_STAT, &sInfo ) ) < 0 )
            perror( "openShm: shmctl()" );
        else
            mShmTotalSize = sInfo.shm_segsz;
    }
}

int checkShm()
{
    if ( !mShmPtr )
        return 0;

    // get a pointer to the shm info block
    RDB_SHM_HDR_t* shmHdr = ( RDB_SHM_HDR_t* ) ( mShmPtr );

    if ( !shmHdr )
        return 0;

    // Tianshu 2021.2.7 comment out
    /*
    if ( ( shmHdr->noBuffers != 2 ) )
    {
        fprintf( stderr, "checkShm: no or wrong number of buffers in shared memory. I need two buffers!" );
        return 0;
    }
    */
    //fprintf( stderr, "no of buffer = %d\n", shmHdr->noBuffers);

    // allocate space for the buffer infos
    RDB_SHM_BUFFER_INFO_t** pBufferInfo = ( RDB_SHM_BUFFER_INFO_t** ) ( new char [ shmHdr->noBuffers * sizeof( RDB_SHM_BUFFER_INFO_t* ) ] );
    RDB_SHM_BUFFER_INFO_t*  pCurrentBufferInfo = 0;

    char* dataPtr = ( char* ) shmHdr;
    dataPtr += shmHdr->headerSize;

    for ( int i = 0; i < shmHdr->noBuffers; i++ )
    {
        pBufferInfo[ i ] = ( RDB_SHM_BUFFER_INFO_t* ) dataPtr;
        dataPtr += pBufferInfo[ i ]->thisSize;
    }

    // get the pointers to message section in each buffer
    RDB_MSG_t* pRdbMsgA = ( RDB_MSG_t* ) ( ( ( char* ) mShmPtr ) + pBufferInfo[0]->offset );
    //RDB_MSG_t* pRdbMsgB = ( RDB_MSG_t* ) ( ( ( char* ) mShmPtr ) + pBufferInfo[1]->offset );
    
    // pointer to the message that will actually be read
    RDB_MSG_t* pRdbMsg  = 0;

    // remember the flags that are set for each buffer
    unsigned int flagsA = pBufferInfo[ 0 ]->flags;
    //unsigned int flagsB = pBufferInfo[ 1 ]->flags;

    // check whether any buffer is ready for reading (checkMask is set (or 0) and buffer is NOT locked)
    bool readyForReadA = ( ( flagsA & mCheckMask ) || !mCheckMask ) && !( flagsA & RDB_SHM_BUFFER_FLAG_LOCK );
    //bool readyForReadB = ( ( flagsB & mCheckMask ) || !mCheckMask ) && !( flagsB & RDB_SHM_BUFFER_FLAG_LOCK );

    if ( mVerbose )
    {
        fprintf( stderr, "ImageReader::checkShm: before processing SHM\n" );
        fprintf( stderr, "ImageReader::checkShm: Buffer A: frameNo = %06d, flags = 0x%x, locked = <%s>, lock mask set = <%s>, readyForRead = <%s>\n", 
                         pRdbMsgA->hdr.frameNo, 
                         flagsA,
                         ( flagsA & RDB_SHM_BUFFER_FLAG_LOCK ) ? "true" : "false",
                         ( flagsA & mCheckMask ) ? "true" : "false",
                         readyForReadA ?  "true" : "false" );

        /*
        fprintf( stderr, "                       Buffer B: frameNo = %06d, flags = 0x%x, locked = <%s>, lock mask set = <%s>, readyForRead = <%s>\n", 
                         pRdbMsgB->hdr.frameNo, 
                         flagsB,
                         ( flagsB & RDB_SHM_BUFFER_FLAG_LOCK ) ? "true" : "false",
                         ( flagsB & mCheckMask ) ? "true" : "false",
                         readyForReadB ?  "true" : "false" );
        */
    }

    if (readyForReadA)
    {
        pRdbMsg = pRdbMsgA;
        pCurrentBufferInfo = pBufferInfo[0];
    }
    
    /*
    if ( mForceBuffer < 0 )  // auto-select the buffer if none is forced to be read
    {
        // check which buffer to read
        if ( ( readyForReadA ) && ( readyForReadB ) )
        {
            if ( pRdbMsgA->hdr.frameNo > pRdbMsgB->hdr.frameNo )        // force using the latest image!!
            {
                pRdbMsg            = pRdbMsgA; 
                pCurrentBufferInfo = pBufferInfo[ 0 ];
            }
            else
            {
                pRdbMsg            = pRdbMsgB; 
                pCurrentBufferInfo = pBufferInfo[ 1 ];
            }
        }
        else if ( readyForReadA )
        {
            pRdbMsg            = pRdbMsgA; 
            pCurrentBufferInfo = pBufferInfo[ 0 ];
        }
        else if ( readyForReadB )
        {
            pRdbMsg            = pRdbMsgB;
            pCurrentBufferInfo = pBufferInfo[ 1 ];
        }
    }
    else if ( ( mForceBuffer == 0 ) && readyForReadA )   // force reading buffer A
    {
        pRdbMsg            = pRdbMsgA; 
        pCurrentBufferInfo = pBufferInfo[ 0 ];
    }
    else if ( ( mForceBuffer == 1 ) && readyForReadB ) // force reading buffer B
    {
        pRdbMsg            = pRdbMsgB;
        pCurrentBufferInfo = pBufferInfo[ 1 ];
    }
    */
    
    // lock the buffer that will be processed now (by this, no other process will alter the contents)
    if ( pCurrentBufferInfo )
        pCurrentBufferInfo->flags |= RDB_SHM_BUFFER_FLAG_LOCK;

    // no data available?
    if ( !pRdbMsg || !pCurrentBufferInfo )
    {
        delete pBufferInfo;
        pBufferInfo = 0;

        // return with valid result if simulation is not yet running
        if ( ( pRdbMsgA->hdr.frameNo == 0 ) ) //&& ( pRdbMsgB->hdr.frameNo == 0 ) )
            return 1;

        // otherwise return a failure
        return 0;
    }

    // handle all messages in the buffer
    if ( !pRdbMsg->hdr.dataSize )
    {
        fprintf( stderr, "checkShm: zero message data size, error.\n" );
        return 0;
    }
    
    
    unsigned int maxReadSize = pCurrentBufferInfo->bufferSize;
    
    while ( 1 )
    {
        // handle the message that is contained in the buffer; this method should be provided by the user (i.e. YOU!)
        handleMessage( pRdbMsg );
        
        // do not read more bytes than there are in the buffer (avoid reading a following buffer accidentally)
        maxReadSize -= pRdbMsg->hdr.dataSize + pRdbMsg->hdr.headerSize;

        if ( maxReadSize < ( pRdbMsg->hdr.headerSize + pRdbMsg->entryHdr.headerSize ) )
            break;
            
        // go to the next message (if available); there may be more than one message in an SHM buffer!
        pRdbMsg = ( RDB_MSG_t* ) ( ( char* ) pRdbMsg + pRdbMsg->hdr.dataSize + pRdbMsg->hdr.headerSize );
        
        if ( !pRdbMsg )
            break;
            
        if ( pRdbMsg->hdr.magicNo != RDB_MAGIC_NO )
            break;
    }
    
    // release after reading
    pCurrentBufferInfo->flags &= ~mCheckMask;                   // remove the check mask
    pCurrentBufferInfo->flags &= ~RDB_SHM_BUFFER_FLAG_LOCK;     // remove the lock mask

    if ( mVerbose )
    {
        unsigned int flagsA = pBufferInfo[ 0 ]->flags;
        //unsigned int flagsB = pBufferInfo[ 1 ]->flags;

        fprintf( stderr, "ImageReader::checkShm: after processing SHM\n" );
        fprintf( stderr, "ImageReader::checkShm: Buffer A: frameNo = %06d, flags = 0x%x, locked = <%s>, lock mask set = <%s>\n", 
                         pRdbMsgA->hdr.frameNo, 
                         flagsA,
                         ( flagsA & RDB_SHM_BUFFER_FLAG_LOCK ) ? "true" : "false",
                         ( flagsA & mCheckMask ) ? "true" : "false" );
        /*
        fprintf( stderr, "                       Buffer B: frameNo = %06d, flags = 0x%x, locked = <%s>, lock mask set = <%s>.\n", 
                         pRdbMsgB->hdr.frameNo, 
                         flagsB,
                         ( flagsB & RDB_SHM_BUFFER_FLAG_LOCK ) ? "true" : "false",
                         ( flagsB & mCheckMask ) ? "true" : "false" );
        */
    }

    return 1;
}    

void handleMessage( RDB_MSG_t* msg )
{
    if ( !msg )
      return;

    if ( !msg->hdr.dataSize )
        return;

    RDB_MSG_ENTRY_HDR_t* entry = ( RDB_MSG_ENTRY_HDR_t* ) ( ( ( char* ) msg ) + msg->hdr.headerSize );
    uint32_t remainingBytes    = msg->hdr.dataSize;

    int msgCounter = 1;
    while ( remainingBytes )
    {
        parseRDBMessageEntry( msg->hdr.simTime, msg->hdr.frameNo, entry, msgCounter);

        remainingBytes -= ( entry->headerSize + entry->dataSize );

        if ( remainingBytes )
          entry = ( RDB_MSG_ENTRY_HDR_t* ) ( ( ( ( char* ) entry ) + entry->headerSize + entry->dataSize ) );
    }
}

void parseRDBMessageEntry( const double & simTime, const unsigned int & simFrame, RDB_MSG_ENTRY_HDR_t* entryHdr, int& counter)
{
    if (!entryHdr)
        return;

    int noElements = entryHdr->elementSize ? ( entryHdr->dataSize / entryHdr->elementSize ) : 0;
    //fprintf(stderr, "Packages found %i \n", noElements);

    char* dataPtr = (char*) entryHdr;

    dataPtr += entryHdr->headerSize;

    while (noElements--)      // only two types of messages are handled here
    {
        switch (entryHdr->pkgId)
        {
            case RDB_PKG_ID_IMAGE:
                fprintf(stderr, "Package type RDB_PKG_ID_IMAGE\n");
                handleRDBitem(simTime, simFrame, (RDB_IMAGE_t*) dataPtr, counter++);
                break;
            case RDB_PKG_ID_CUSTOM_OPTIX_START:
                fprintf(stderr, "Package type RDB_PKG_ID_CUSTOM_OPTIX_START\n");
                handleRDBitem(simTime, simFrame, (RDB_IMAGE_t*) dataPtr, counter++);
                break;

            default:
                //fprintf( stderr, "Unsupported package type %u \n", entryHdr->pkgId);
                break;
        }

        dataPtr += entryHdr->elementSize;
     }
}

void handleRDBitem( const double & simTime, const unsigned int & simFrame, RDB_IMAGE_t* msgImage, int counter)
{
    fprintf( stderr, "handleRDBitem: called" );
    
    if ( !msgImage )
        return;

    fprintf( stderr, "handleRDBitem: image\n" );
    fprintf( stderr, "    simTime = %.3lf, simFrame = %ld\n", simTime, simFrame);
    fprintf( stderr, "    width / height = %d / %d\n", msgImage->width, msgImage->height );
    fprintf( stderr, "    dataSize = %d\n", msgImage->imgSize );
    fprintf( stderr, "    RDB_PIX_FORMAT = %hu\n", msgImage->pixelFormat );


    // Tianshu 2021.2.7 RGB8
    if (msgImage->pixelFormat != RDB_PIX_FORMAT_RGB8) 
    {
        std::cerr << "dumpBufferStructAsFile: Only pixelformat RDB_PIX_FORMAT_RGBA32F supported for writing files. Aborting writing file. " << msgImage->pixelFormat << std::endl;
        return;
    }

    // Tianshu 2021.2.7 
    static const int NUMBER_OF_VALUES = 3;  //4;

    int width = msgImage->width*NUMBER_OF_VALUES;
    int height = msgImage->height;
    
    // PCD
//    std::stringstream sstrFileNamePCD;
//    sstrFileNamePCD << "/tmp/point_cloud_frame_" << simFrame << "_time_" << simTime << "_" << counter << ".pcd";
//    std::ofstream filePCD(sstrFileNamePCD.str().c_str());
    
    // Extra Data
//    std::stringstream sstrFileNameData;
//    sstrFileNameData << "/tmp/data_frame_" << simFrame << "_time_" << simTime << "_" << counter << ".txt";
//    std::ofstream fileData(sstrFileNameData.str().c_str());
    
    // Tianshu 2021.2.7 IMAGE
    //RGB
    /* std::stringstream sstrFileNameIMG;
    sstrFileNameIMG << "image_frame_" << simFrame << "_time_" << simTime << "_" << counter << ".rgb";
    std::ofstream fileIMG(sstrFileNameIMG.str().c_str());
    
    fprintf( stderr, "ZOU: prepare hdr");
    char rgbHdr[512];
    memset ( rgbHdr, 0x0, 512);
    
    rgbHdr[0] = 0x01;
    rgbHdr[1] = 0xda;   // MGAIC
    rgbHdr[2] = 0x00;   // uncompressed
    rgbHdr[3] = 0x01;   // 1 byte per channel
    rgbHdr[4] = 0x00;   
    rgbHdr[5] = 0x03;   // dimension
    // rgbHdr[6] = 0x03;   // X 960
    // rgbHdr[7] = 0xC0;
    // rgbHdr[8] = 0x02;   // Y 540; 570?
    // rgbHdr[9] = 0x3A;
    rgbHdr[6] = 0x03;   // X 800
    rgbHdr[7] = 0x20;
    rgbHdr[8] = 0x02;   // Y 600?
    rgbHdr[9] = 0x58;
    rgbHdr[10] = 0x00;
    rgbHdr[11] = 0x03;  // Channel
    rgbHdr[20] = 0xff;  // ?
    fprintf( stderr, "ZOU: write rgb header");
    fileIMG.write( rgbHdr, 512 );


    // char *data = (char*)(msgImage+1);
    // fprintf( stderr, "ZOU: write rgb data ");
    // fileIMG.write( data, msgImage->imgSize );

    
    // filePCD << "# .PCD v.7 - Point Cloud Data file format" << "\n";
    // filePCD <<  "VERSION .7"   << "\n";
    // filePCD <<  "FIELDS x y z" << "\n";
    // filePCD <<  "SIZE 4 4 4"   << "\n";
    // filePCD <<  "TYPE F F F"   << "\n";
    // filePCD <<  "COUNT 1 1 1"  << "\n";
    // filePCD <<  "WIDTH "  << msgImage->width  << "\n";
    // filePCD <<  "HEIGHT " << msgImage->height << "\n";
    // filePCD <<  "VIEWPOINT 0 0 0 1 0 0 0"      << "\n";
    // filePCD <<  "POINTS " << (msgImage->width * msgImage->height)  << "\n";
    // filePCD <<  "DATA ascii" << "\n";
    // filePCD <<  "" << "\n";
   
    
    // RED
    for (int y = 0; y < height ; y++)
    {
        for (int x = 0; x < width; x += NUMBER_OF_VALUES)
        {
            // Tianshu 2021.2.7 IMAGE
            char r = reinterpret_cast< char*>( msgImage + 1 )[x + (y * width)];        
            fileIMG.write( &r, 1 );
        }
    }
    // GREEN
    for (int y = 0; y < height ; y++)
    {
        for (int x = 0; x < width; x += NUMBER_OF_VALUES)
        {
            char g = reinterpret_cast< char*>( msgImage + 1 )[x + (y * width + 1)];
            fileIMG.write( &g, 1 );
        }
    }
    // BLUE
    for (int y = 0; y < height ; y++)
    {
        for (int x = 0; x < width; x += NUMBER_OF_VALUES)
        {
            char b = reinterpret_cast< char*>( msgImage + 1 )[x + (y * width + 2)];
            fileIMG.write( &b, 1 );
        }
    } */
 
//OPENCV方式
    char *data = (char*)(msgImage+1);
    // fprintf( stderr, "ZOU: write rgb data ");
    // fileIMG.write( data, msgImage->imgSize );
    cv::Mat YUVImg(height, width/3, CV_8UC3,(unsigned char*)data);
    if(YUVImg.data == nullptr){
        std::cout << "[CallbackCameraImage_cdds]  receiver img null!"  << std::endl;
        return;
        }
    std::stringstream sstrFileNameIMG;
    sstrFileNameIMG << "image_frame_" << simFrame << "_time_" << simTime << "_" << counter << ".png";
    cv::Mat RGBImg(height,width/3, CV_8UC3);
    cv::cvtColor(YUVImg, RGBImg, cv::COLOR_RGB2BGR);
    cv::Mat RGBImg_rotate(height,width/3, CV_8UC3);
    cv::Mat RGBImg_flip(height,width/3, CV_8UC3);
    // cv::imwrite(sstrFileNameIMG.str().c_str(),  RGBImg);
    // cv::rotate(RGBImg, RGBImg_rotate, cv::ROTATE_180);
    // cv::imwrite(sstrFileNameIMG.str().c_str(),  RGBImg_rotate);
    cv::flip(RGBImg, RGBImg_flip,0);
    cv::imwrite(sstrFileNameIMG.str().c_str(),  RGBImg_flip);


    

    
}

