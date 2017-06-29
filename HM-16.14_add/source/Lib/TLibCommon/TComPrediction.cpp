/* The copyright in this software is being made available under the BSD
 * License, included below. This software may be subject to other third party
 * and contributor rights, including patent rights, and no such rights are
 * granted under this license.
 *
 * Copyright (c) 2010-2016, ITU/ISO/IEC
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *  * Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 *  * Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *  * Neither the name of the ITU/ISO/IEC nor the names of its contributors may
 *    be used to endorse or promote products derived from this software without
 *    specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS
 * BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF
 * THE POSSIBILITY OF SUCH DAMAGE.
 */

/** \file     TComPrediction.cpp
    \brief    prediction class
*/

#include <memory.h>
#include "TComPrediction.h"
#include "TComPic.h"
#include "TComTU.h"

//! \ingroup TLibCommon
//! \{

// ====================================================================================================================
// Tables
// ====================================================================================================================

const UChar TComPrediction::m_aucIntraFilter[MAX_NUM_CHANNEL_TYPE][MAX_INTRA_FILTER_DEPTHS] =
{
  { // Luma
    10, //4x4
    7, //8x8
    1, //16x16
    0, //32x32
    10, //64x64
  },
  { // Chroma
    10, //4xn
    7, //8xn
    1, //16xn
    0, //32xn
    10, //64xn
  }

};

// ====================================================================================================================
// Constructor / destructor / initialize
// ====================================================================================================================

TComPrediction::TComPrediction()
: m_pLumaRecBuffer(0)
, m_iLumaRecStride(0)
{
  for(UInt ch=0; ch<MAX_NUM_COMPONENT; ch++)
  {
    for(UInt buf=0; buf<2; buf++)
    {
      m_piYuvExt[ch][buf] = NULL;
    }
  }
}

TComPrediction::~TComPrediction()
{
  destroy();
}

Void TComPrediction::destroy()
{
  for(UInt ch=0; ch<MAX_NUM_COMPONENT; ch++)
  {
    for(UInt buf=0; buf<NUM_PRED_BUF; buf++)
    {
      delete [] m_piYuvExt[ch][buf];
      m_piYuvExt[ch][buf] = NULL;
    }
  }

  for(UInt i=0; i<NUM_REF_PIC_LIST_01; i++)
  {
    m_acYuvPred[i].destroy();
  }

  m_cYuvPredTemp.destroy();

  if( m_pLumaRecBuffer )
  {
    delete [] m_pLumaRecBuffer;
    m_pLumaRecBuffer = 0;
  }
  m_iLumaRecStride = 0;

  for (UInt i = 0; i < LUMA_INTERPOLATION_FILTER_SUB_SAMPLE_POSITIONS; i++)
  {
    for (UInt j = 0; j < LUMA_INTERPOLATION_FILTER_SUB_SAMPLE_POSITIONS; j++)
    {
      m_filteredBlock[i][j].destroy();
    }
    m_filteredBlockTmp[i].destroy();
  }
}

Void TComPrediction::initTempBuff(ChromaFormat chromaFormatIDC)
{
  // if it has been initialised before, but the chroma format has changed, release the memory and start again.
  if( m_piYuvExt[COMPONENT_Y][PRED_BUF_UNFILTERED] != NULL && m_cYuvPredTemp.getChromaFormat()!=chromaFormatIDC)
  {
    destroy();
  }

  if( m_piYuvExt[COMPONENT_Y][PRED_BUF_UNFILTERED] == NULL ) // check if first is null (in which case, nothing initialised yet)
  {
    Int extWidth  = MAX_CU_SIZE + 16;
    Int extHeight = MAX_CU_SIZE + 1;

    for (UInt i = 0; i < LUMA_INTERPOLATION_FILTER_SUB_SAMPLE_POSITIONS; i++)
    {
      m_filteredBlockTmp[i].create(extWidth, extHeight + 7, chromaFormatIDC);
      for (UInt j = 0; j < LUMA_INTERPOLATION_FILTER_SUB_SAMPLE_POSITIONS; j++)
      {
        m_filteredBlock[i][j].create(extWidth, extHeight, chromaFormatIDC);
      }
    }

    m_iYuvExtSize = (MAX_CU_SIZE*2+1) * (MAX_CU_SIZE*2+1);
    for(UInt ch=0; ch<MAX_NUM_COMPONENT; ch++)
    {
      for(UInt buf=0; buf<NUM_PRED_BUF; buf++)
      {
        m_piYuvExt[ch][buf] = new Pel[ m_iYuvExtSize ];
      }
    }

    // new structure
    for(UInt i=0; i<NUM_REF_PIC_LIST_01; i++)
    {
      m_acYuvPred[i] .create( MAX_CU_SIZE, MAX_CU_SIZE, chromaFormatIDC );
    }

    m_cYuvPredTemp.create( MAX_CU_SIZE, MAX_CU_SIZE, chromaFormatIDC );
  }


  if (m_iLumaRecStride != (MAX_CU_SIZE>>1) + 1)
  {
    m_iLumaRecStride =  (MAX_CU_SIZE>>1) + 1;
    if (!m_pLumaRecBuffer)
    {
      m_pLumaRecBuffer = new Pel[ m_iLumaRecStride * m_iLumaRecStride ];
    }
  }
}

// ====================================================================================================================
// Public member functions
// ====================================================================================================================

// Function for calculating DC value of the reference samples used in Intra prediction
//NOTE: Bit-Limit - 25-bit source
Pel TComPrediction::predIntraGetPredValDC( const Pel* pSrc, Int iSrcStride, UInt iWidth, UInt iHeight)
{
  assert(iWidth > 0 && iHeight > 0);
  Int iInd, iSum = 0;
  Pel pDcVal;

  for (iInd = 0;iInd < iWidth;iInd++)
  {
    iSum += pSrc[iInd-iSrcStride];
  }
  for (iInd = 0;iInd < iHeight;iInd++)
  {
    iSum += pSrc[iInd*iSrcStride-1];
  }

  pDcVal = (iSum + iWidth) / (iWidth + iHeight);

  return pDcVal;
}

// Function for deriving the angular Intra predictions

/** Function for deriving the simplified angular intra predictions.
 * \param bitDepth           bit depth
 * \param pSrc               pointer to reconstructed sample array
 * \param srcStride          the stride of the reconstructed sample array
 * \param pTrueDst           reference to pointer for the prediction sample array
 * \param dstStrideTrue      the stride of the prediction sample array
 * \param uiWidth            the width of the block
 * \param uiHeight           the height of the block
 * \param channelType        type of pel array (luma/chroma)
 * \param format             chroma format
 * \param dirMode            the intra prediction mode index
 * \param blkAboveAvailable  boolean indication if the block above is available
 * \param blkLeftAvailable   boolean indication if the block to the left is available
 * \param bEnableEdgeFilters indication whether to enable edge filters
 *
 * This function derives the prediction samples for the angular mode based on the prediction direction indicated by
 * the prediction mode index. The prediction direction is given by the displacement of the bottom row of the block and
 * the reference row above the block in the case of vertical prediction or displacement of the rightmost column
 * of the block and reference column left from the block in the case of the horizontal prediction. The displacement
 * is signalled at 1/32 pixel accuracy. When projection of the predicted pixel falls inbetween reference samples,
 * the predicted value for the pixel is linearly interpolated from the reference samples. All reference samples are taken
 * from the extended main reference.
 */
//NOTE: Bit-Limit - 25-bit source
Void TComPrediction::xPredIntraAng(       Int bitDepth,
                                    const Pel* pSrc,     Int srcStride,
                                          Pel* pTrueDst, Int dstStrideTrue,
                                          UInt uiWidth, UInt uiHeight, ChannelType channelType,
                                          UInt dirMode, const Bool bEnableEdgeFilters
                                  )
{
  Int width=Int(uiWidth);
  Int height=Int(uiHeight);

  // Map the mode index to main prediction direction and angle
  assert( dirMode != PLANAR_IDX ); //no planar
  const Bool modeDC        = dirMode==DC_IDX;

  // Do the DC prediction
  if (modeDC)
  {
    const Pel dcval = predIntraGetPredValDC(pSrc, srcStride, width, height);

    for (Int y=height;y>0;y--, pTrueDst+=dstStrideTrue)
    {
      for (Int x=0; x<width;) // width is always a multiple of 4.
      {
        pTrueDst[x++] = dcval;
      }
    }
  }
  else // Do angular predictions
  {
    const Bool       bIsModeVer         = (dirMode >= 18);
    const Int        intraPredAngleMode = (bIsModeVer) ? (Int)dirMode - VER_IDX :  -((Int)dirMode - HOR_IDX);
    const Int        absAngMode         = abs(intraPredAngleMode);
    const Int        signAng            = intraPredAngleMode < 0 ? -1 : 1;
    const Bool       edgeFilter         = bEnableEdgeFilters && isLuma(channelType) && (width <= MAXIMUM_INTRA_FILTERED_WIDTH) && (height <= MAXIMUM_INTRA_FILTERED_HEIGHT);

    // Set bitshifts and scale the angle parameter to block size
    static const Int angTable[9]    = {0,    2,    5,   9,  13,  17,  21,  26,  32};
    static const Int invAngTable[9] = {0, 4096, 1638, 910, 630, 482, 390, 315, 256}; // (256 * 32) / Angle
    Int invAngle                    = invAngTable[absAngMode];
    Int absAng                      = angTable[absAngMode];
    Int intraPredAngle              = signAng * absAng;

    Pel* refMain;
    Pel* refSide;

    Pel  refAbove[2*MAX_CU_SIZE+1];
    Pel  refLeft[2*MAX_CU_SIZE+1];

    // Initialize the Main and Left reference array.
    if (intraPredAngle < 0)
    {
      const Int refMainOffsetPreScale = (bIsModeVer ? height : width ) - 1;
      const Int refMainOffset         = height - 1;
      for (Int x=0;x<width+1;x++)
      {
        refAbove[x+refMainOffset] = pSrc[x-srcStride-1];
      }
      for (Int y=0;y<height+1;y++)
      {
        refLeft[y+refMainOffset] = pSrc[(y-1)*srcStride-1];
      }
      refMain = (bIsModeVer ? refAbove : refLeft)  + refMainOffset;
      refSide = (bIsModeVer ? refLeft  : refAbove) + refMainOffset;

      // Extend the Main reference to the left.
      Int invAngleSum    = 128;       // rounding for (shift by 8)
      for (Int k=-1; k>(refMainOffsetPreScale+1)*intraPredAngle>>5; k--)
      {
        invAngleSum += invAngle;
        refMain[k] = refSide[invAngleSum>>8];
      }
    }
    else
    {
      for (Int x=0;x<2*width+1;x++)
      {
        refAbove[x] = pSrc[x-srcStride-1];
      }
      for (Int y=0;y<2*height+1;y++)
      {
        refLeft[y] = pSrc[(y-1)*srcStride-1];
      }
      refMain = bIsModeVer ? refAbove : refLeft ;
      refSide = bIsModeVer ? refLeft  : refAbove;
    }

    // swap width/height if we are doing a horizontal mode:
    Pel tempArray[MAX_CU_SIZE*MAX_CU_SIZE];
    const Int dstStride = bIsModeVer ? dstStrideTrue : MAX_CU_SIZE;
    Pel *pDst = bIsModeVer ? pTrueDst : tempArray;
    if (!bIsModeVer)
    {
      std::swap(width, height);
    }

    if (intraPredAngle == 0)  // pure vertical or pure horizontal
    {
      for (Int y=0;y<height;y++)
      {
        for (Int x=0;x<width;x++)
        {
          pDst[y*dstStride+x] = refMain[x+1];
        }
      }

      if (edgeFilter)
      {
        for (Int y=0;y<height;y++)
        {
          pDst[y*dstStride] = Clip3 (0, ((1 << bitDepth) - 1), pDst[y*dstStride] + (( refSide[y+1] - refSide[0] ) >> 1) );
        }
      }
    }
    else
    {
      Pel *pDsty=pDst;

      for (Int y=0, deltaPos=intraPredAngle; y<height; y++, deltaPos+=intraPredAngle, pDsty+=dstStride)
      {
        const Int deltaInt   = deltaPos >> 5;
        const Int deltaFract = deltaPos & (32 - 1);

        if (deltaFract)
        {
          // Do linear filtering
          const Pel *pRM=refMain+deltaInt+1;
          Int lastRefMainPel=*pRM++;
          for (Int x=0;x<width;pRM++,x++)
          {
            Int thisRefMainPel=*pRM;
            pDsty[x+0] = (Pel) ( ((32-deltaFract)*lastRefMainPel + deltaFract*thisRefMainPel +16) >> 5 );
            lastRefMainPel=thisRefMainPel;
          }
        }
        else
        {
          // Just copy the integer samples
          for (Int x=0;x<width; x++)
          {
            pDsty[x] = refMain[x+deltaInt+1];
          }
        }
      }
    }

    // Flip the block if this is the horizontal mode
    if (!bIsModeVer)
    {
      for (Int y=0; y<height; y++)
      {
        for (Int x=0; x<width; x++)
        {
          pTrueDst[x*dstStrideTrue] = pDst[x];
        }
        pTrueDst++;
        pDst+=dstStride;
      }
    }
  }
}

Void TComPrediction::predIntraAng( const ComponentID compID, UInt uiDirMode, Pel* piOrg /* Will be null for decoding */, UInt uiOrgStride, Pel* piPred, UInt uiStride, TComTU &rTu, const Bool bUseFilteredPredSamples, const Bool bUseLosslessDPCM )
{
  const ChannelType    channelType = toChannelType(compID);
  const TComRectangle &rect        = rTu.getRect(isLuma(compID) ? COMPONENT_Y : COMPONENT_Cb);
  const Int            iWidth      = rect.width;
  const Int            iHeight     = rect.height;

  assert( g_aucConvertToBit[ iWidth ] >= 0 ); //   4x  4
  assert( g_aucConvertToBit[ iWidth ] <= 5 ); // 128x128
  //assert( iWidth == iHeight  );

        Pel *pDst = piPred;

  // get starting pixel in block
  const Int sw = (2 * iWidth + 1);

  if ( bUseLosslessDPCM )
  {
    const Pel *ptrSrc = getPredictorPtr( compID, false );
    // Sample Adaptive intra-Prediction (SAP)
    if (uiDirMode==HOR_IDX)
    {
      // left column filled with reference samples
      // remaining columns filled with piOrg data (if available).
      for(Int y=0; y<iHeight; y++)
      {
        piPred[y*uiStride+0] = ptrSrc[(y+1)*sw];
      }
      if (piOrg!=0)
      {
        piPred+=1; // miss off first column
        for(Int y=0; y<iHeight; y++, piPred+=uiStride, piOrg+=uiOrgStride)
        {
          memcpy(piPred, piOrg, (iWidth-1)*sizeof(Pel));
        }
      }
    }
    else // VER_IDX
    {
      // top row filled with reference samples
      // remaining rows filled with piOrd data (if available)
      for(Int x=0; x<iWidth; x++)
      {
        piPred[x] = ptrSrc[x+1];
      }
      if (piOrg!=0)
      {
        piPred+=uiStride; // miss off the first row
        for(Int y=1; y<iHeight; y++, piPred+=uiStride, piOrg+=uiOrgStride)
        {
          memcpy(piPred, piOrg, iWidth*sizeof(Pel));
        }
      }
    }
  }
  else
  {
    const Pel *ptrSrc = getPredictorPtr( compID, bUseFilteredPredSamples );

    if ( uiDirMode == PLANAR_IDX )
    {
      xPredIntraPlanar( ptrSrc+sw+1, sw, pDst, uiStride, iWidth, iHeight );
    }
    else
    {
      // Create the prediction
            TComDataCU *const pcCU              = rTu.getCU();
      const UInt              uiAbsPartIdx      = rTu.GetAbsPartIdxTU();
      const Bool              enableEdgeFilters = !(pcCU->isRDPCMEnabled(uiAbsPartIdx) && pcCU->getCUTransquantBypass(uiAbsPartIdx));
#if O0043_BEST_EFFORT_DECODING
      const Int channelsBitDepthForPrediction = rTu.getCU()->getSlice()->getSPS()->getStreamBitDepth(channelType);
#else
      const Int channelsBitDepthForPrediction = rTu.getCU()->getSlice()->getSPS()->getBitDepth(channelType);
#endif
      xPredIntraAng( channelsBitDepthForPrediction, ptrSrc+sw+1, sw, pDst, uiStride, iWidth, iHeight, channelType, uiDirMode, enableEdgeFilters );

      if( uiDirMode == DC_IDX )
      {
        xDCPredFiltering( ptrSrc+sw+1, sw, pDst, uiStride, iWidth, iHeight, channelType );
      }
    }
  }

}

/** Check for identical motion in both motion vector direction of a bi-directional predicted CU
  * \returns true, if motion vectors and reference pictures match
 */
Bool TComPrediction::xCheckIdenticalMotion ( TComDataCU* pcCU, UInt PartAddr )
{
  if( pcCU->getSlice()->isInterB() && !pcCU->getSlice()->getPPS()->getWPBiPred() )
  {
    if( pcCU->getCUMvField(REF_PIC_LIST_0)->getRefIdx(PartAddr) >= 0 && pcCU->getCUMvField(REF_PIC_LIST_1)->getRefIdx(PartAddr) >= 0)
    {
      Int RefPOCL0 = pcCU->getSlice()->getRefPic(REF_PIC_LIST_0, pcCU->getCUMvField(REF_PIC_LIST_0)->getRefIdx(PartAddr))->getPOC();
      Int RefPOCL1 = pcCU->getSlice()->getRefPic(REF_PIC_LIST_1, pcCU->getCUMvField(REF_PIC_LIST_1)->getRefIdx(PartAddr))->getPOC();
      if(RefPOCL0 == RefPOCL1 && pcCU->getCUMvField(REF_PIC_LIST_0)->getMv(PartAddr) == pcCU->getCUMvField(REF_PIC_LIST_1)->getMv(PartAddr))
      {
        return true;
      }
    }
  }
  return false;
}



Void TComPrediction::motionCompensation ( TComDataCU* pcCU, TComYuv* pcYuvPred, RefPicList eRefPicList, Int iPartIdx )
{
  Int         iWidth;
  Int         iHeight;
  UInt        uiPartAddr;
  const TComSlice *pSlice    = pcCU->getSlice();
  const SliceType  sliceType = pSlice->getSliceType();
  const TComPPS   &pps       = *(pSlice->getPPS());

  if ( iPartIdx >= 0 )
  {
    // 得到当前PU的索引和大小 
    pcCU->getPartIndexAndSize( iPartIdx, uiPartAddr, iWidth, iHeight );
    if ( eRefPicList != REF_PIC_LIST_X )
    {
      if( (sliceType == P_SLICE && pps.getUseWP()) || (sliceType == B_SLICE && pps.getWPBiPred()))
      {
        xPredInterUni (pcCU, uiPartAddr, iWidth, iHeight, eRefPicList, pcYuvPred, true );
        xWeightedPredictionUni( pcCU, pcYuvPred, uiPartAddr, iWidth, iHeight, eRefPicList, pcYuvPred );
      }
      else
      {
        xPredInterUni (pcCU, uiPartAddr, iWidth, iHeight, eRefPicList, pcYuvPred );
      }
    }
    else
    {
      if ( xCheckIdenticalMotion( pcCU, uiPartAddr ) )
      {
        xPredInterUni (pcCU, uiPartAddr, iWidth, iHeight, REF_PIC_LIST_0, pcYuvPred );
      }
      else
      {
        xPredInterBi  (pcCU, uiPartAddr, iWidth, iHeight, pcYuvPred );
      }
    }
    return;
  }

  for ( iPartIdx = 0; iPartIdx < pcCU->getNumPartitions(); iPartIdx++ )
  {
    pcCU->getPartIndexAndSize( iPartIdx, uiPartAddr, iWidth, iHeight );

    if ( eRefPicList != REF_PIC_LIST_X )
    {
      if( (sliceType == P_SLICE && pps.getUseWP()) || (sliceType == B_SLICE && pps.getWPBiPred()))
      {
        xPredInterUni (pcCU, uiPartAddr, iWidth, iHeight, eRefPicList, pcYuvPred, true );
        xWeightedPredictionUni( pcCU, pcYuvPred, uiPartAddr, iWidth, iHeight, eRefPicList, pcYuvPred );
      }
      else
      {
        xPredInterUni (pcCU, uiPartAddr, iWidth, iHeight, eRefPicList, pcYuvPred );
      }
    }
    else
    {
      if ( xCheckIdenticalMotion( pcCU, uiPartAddr ) )
      {
        xPredInterUni (pcCU, uiPartAddr, iWidth, iHeight, REF_PIC_LIST_0, pcYuvPred );
      }
      else
      {
        xPredInterBi  (pcCU, uiPartAddr, iWidth, iHeight, pcYuvPred );
      }
    }
  }
  return;
}

Void TComPrediction::xPredInterUni ( TComDataCU* pcCU, UInt uiPartAddr, Int iWidth, Int iHeight, RefPicList eRefPicList, TComYuv* pcYuvPred, Bool bi )
{
  Int         iRefIdx     = pcCU->getCUMvField( eRefPicList )->getRefIdx( uiPartAddr );           assert (iRefIdx >= 0);
  TComMv      cMv         = pcCU->getCUMvField( eRefPicList )->getMv( uiPartAddr );
  pcCU->clipMv(cMv);
  //Bool bmergeflag = pcCU->getMergeFlag(pcCU->getZorderIdxInCtu() + uiPartAddr);

#if HUANGFU_2017_04_27
  if (!cMv.getPos())
  {
#if HUANGFU_20170606
	  if (pcCU->componenttype)

	  {
		  const ComponentID compID = ComponentID(COMPONENT_Y);
#if ZhengRuidi_20170422
		  xPredInterBlk_Top(compID, pcCU, pcCU->getSlice()->getRefPic(eRefPicList, iRefIdx)->getPicYuvRec(), uiPartAddr, &cMv, iWidth, iHeight, pcYuvPred, bi, pcCU->getSlice()->getSPS()->getBitDepth(toChannelType(compID)));
#else
		  xPredInterBlk(compID, pcCU, pcCU->getSlice()->getRefPic(eRefPicList, iRefIdx)->getPicYuvRec(), uiPartAddr, &cMv, iWidth, iHeight, pcYuvPred, bi, pcCU->getSlice()->getSPS()->getBitDepth(toChannelType(compID)));
#endif

	  }
	  else
#endif
	  for (UInt comp = COMPONENT_Y; comp<pcYuvPred->getNumberValidComponents(); comp++)
	  {
		  const ComponentID compID=ComponentID(comp);

#if ZhengRuidi_20170422
			  xPredInterBlk_Top(compID, pcCU, pcCU->getSlice()->getRefPic(eRefPicList, iRefIdx)->getPicYuvRec(), uiPartAddr, &cMv, iWidth, iHeight, pcYuvPred, bi, pcCU->getSlice()->getSPS()->getBitDepth(toChannelType(compID)));
#else
		  xPredInterBlk(compID, pcCU, pcCU->getSlice()->getRefPic(eRefPicList, iRefIdx)->getPicYuvRec(), uiPartAddr, &cMv, iWidth, iHeight, pcYuvPred, bi, pcCU->getSlice()->getSPS()->getBitDepth(toChannelType(compID)));
#endif
	  }
  }
  else
  {	 
		  Int iWidth_ = 4;
		  Int iHeight_ = 4;
#if HUANGFU_20170606
		  if (pcCU->componenttype)
		  {
			  const ComponentID compID = ComponentID(COMPONENT_Y);
			  for (int i = 0; i < iHeight / 4; i++)
			  {
				  for (int j = 0; j < iWidth / 4; j++)
				  {
					  cMv = pcCU->getCUMvField(eRefPicList)->getMv(g_auiRasterToZscan[g_auiZscanToRaster[uiPartAddr + pcCU->getZorderIdxInCtu()] + j + 16 * i] - pcCU->getZorderIdxInCtu());
					  pcCU->clipMv(cMv);
					  int uiPartAddr_ = g_auiRasterToZscan[g_auiZscanToRaster[uiPartAddr + pcCU->getZorderIdxInCtu()] + j + 16 * i] - pcCU->getZorderIdxInCtu();

#if ZhengRuidi_20170422
					  xPredInterBlk_Top(compID, pcCU, pcCU->getSlice()->getRefPic(eRefPicList, iRefIdx)->getPicYuvRec(), uiPartAddr_, &cMv, iWidth_, iHeight_, pcYuvPred, bi, pcCU->getSlice()->getSPS()->getBitDepth(toChannelType(compID)));
#else
					  xPredInterBlk(compID, pcCU, pcCU->getSlice()->getRefPic(eRefPicList, iRefIdx)->getPicYuvRec(), uiPartAddr_, &cMv, iWidth_, iHeight_, pcYuvPred, bi, pcCU->getSlice()->getSPS()->getBitDepth(toChannelType(compID)));
#endif				  
				  }
			  }
		  }
		  else
#endif
		  {
			  for (UInt comp = COMPONENT_Y; comp < pcYuvPred->getNumberValidComponents(); comp++)
			  {
				  const ComponentID compID = ComponentID(comp);
				  for (int i = 0; i < iHeight / 4; i++)
				  {
					  for (int j = 0; j < iWidth / 4; j++)
					  {
						  cMv = pcCU->getCUMvField(eRefPicList)->getMv(g_auiRasterToZscan[g_auiZscanToRaster[uiPartAddr + pcCU->getZorderIdxInCtu()] + j + 16 * i] - pcCU->getZorderIdxInCtu());
						  pcCU->clipMv(cMv);
						  int uiPartAddr_ = g_auiRasterToZscan[g_auiZscanToRaster[uiPartAddr + pcCU->getZorderIdxInCtu()] + j + 16 * i] - pcCU->getZorderIdxInCtu();

#if ZhengRuidi_20170422
						  xPredInterBlk_Top(compID, pcCU, pcCU->getSlice()->getRefPic(eRefPicList, iRefIdx)->getPicYuvRec(), uiPartAddr_, &cMv, iWidth_, iHeight_, pcYuvPred, bi, pcCU->getSlice()->getSPS()->getBitDepth(toChannelType(compID)));
#else
						  xPredInterBlk(compID, pcCU, pcCU->getSlice()->getRefPic(eRefPicList, iRefIdx)->getPicYuvRec(), uiPartAddr_, &cMv, iWidth_, iHeight_, pcYuvPred, bi, pcCU->getSlice()->getSPS()->getBitDepth(toChannelType(compID)));
#endif				  
					  }
				  }
			  }
		  }
	  
	  
  }
#else
  for (UInt comp=COMPONENT_Y; comp<pcYuvPred->getNumberValidComponents(); comp++)
  {
    const ComponentID compID=ComponentID(comp);
#if ZhengRuidi_20170422
	xPredInterBlk_Top(compID, pcCU, pcCU->getSlice()->getRefPic(eRefPicList, iRefIdx)->getPicYuvRec(), uiPartAddr, &cMv, iWidth, iHeight, pcYuvPred, bi, pcCU->getSlice()->getSPS()->getBitDepth(toChannelType(compID)));

#else
	xPredInterBlk(compID, pcCU, pcCU->getSlice()->getRefPic(eRefPicList, iRefIdx)->getPicYuvRec(), uiPartAddr, &cMv, iWidth, iHeight, pcYuvPred, bi, pcCU->getSlice()->getSPS()->getBitDepth(toChannelType(compID)));
#endif

  }
#endif
}

Void TComPrediction::xPredInterBi ( TComDataCU* pcCU, UInt uiPartAddr, Int iWidth, Int iHeight, TComYuv* pcYuvPred )
{
  TComYuv* pcMbYuv;
  Int      iRefIdx[NUM_REF_PIC_LIST_01] = {-1, -1};

  for ( UInt refList = 0; refList < NUM_REF_PIC_LIST_01; refList++ )
  {
    RefPicList eRefPicList = (refList ? REF_PIC_LIST_1 : REF_PIC_LIST_0);
    iRefIdx[refList] = pcCU->getCUMvField( eRefPicList )->getRefIdx( uiPartAddr );

    if ( iRefIdx[refList] < 0 )
    {
      continue;
    }

    assert( iRefIdx[refList] < pcCU->getSlice()->getNumRefIdx(eRefPicList) );

    pcMbYuv = &m_acYuvPred[refList];
    if( pcCU->getCUMvField( REF_PIC_LIST_0 )->getRefIdx( uiPartAddr ) >= 0 && pcCU->getCUMvField( REF_PIC_LIST_1 )->getRefIdx( uiPartAddr ) >= 0 )
    {
      xPredInterUni ( pcCU, uiPartAddr, iWidth, iHeight, eRefPicList, pcMbYuv, true );
    }
    else
    {
      if ( ( pcCU->getSlice()->getPPS()->getUseWP()       && pcCU->getSlice()->getSliceType() == P_SLICE ) ||
           ( pcCU->getSlice()->getPPS()->getWPBiPred()    && pcCU->getSlice()->getSliceType() == B_SLICE ) )
      {
        xPredInterUni ( pcCU, uiPartAddr, iWidth, iHeight, eRefPicList, pcMbYuv, true );
      }
      else
      {
        xPredInterUni ( pcCU, uiPartAddr, iWidth, iHeight, eRefPicList, pcMbYuv );
      }
    }
  }

  if ( pcCU->getSlice()->getPPS()->getWPBiPred()    && pcCU->getSlice()->getSliceType() == B_SLICE  )
  {
    xWeightedPredictionBi( pcCU, &m_acYuvPred[REF_PIC_LIST_0], &m_acYuvPred[REF_PIC_LIST_1], iRefIdx[REF_PIC_LIST_0], iRefIdx[REF_PIC_LIST_1], uiPartAddr, iWidth, iHeight, pcYuvPred );
  }
  else if ( pcCU->getSlice()->getPPS()->getUseWP() && pcCU->getSlice()->getSliceType() == P_SLICE )
  {
    xWeightedPredictionUni( pcCU, &m_acYuvPred[REF_PIC_LIST_0], uiPartAddr, iWidth, iHeight, REF_PIC_LIST_0, pcYuvPred );
  }
  else
  {
    xWeightedAverage( &m_acYuvPred[REF_PIC_LIST_0], &m_acYuvPred[REF_PIC_LIST_1], iRefIdx[REF_PIC_LIST_0], iRefIdx[REF_PIC_LIST_1], uiPartAddr, iWidth, iHeight, pcYuvPred, pcCU->getSlice()->getSPS()->getBitDepths() );
  }
}

#if ZhengRuidi_20170315 

#if ZhengRuidi_20170319
Void TComPrediction::xPredInterBlk_Top(const ComponentID compID, TComDataCU *cu, TComPicYuv *refPic, UInt partAddr, TComMv *mv, Int width, Int height, TComYuv *dstPic, Bool bi, const Int bitDepth)
{
	assert(height >= 0 && width >= 0);
	Int shiftHor = (2 + refPic->getComponentScaleX(compID));
	Int shiftVer = (2 + refPic->getComponentScaleY(compID));
	Int RotationIdx = 0;
	Int CtuRsAddr = cu->getCtuRsAddr();  //grab CTU raster scan order
	Int CtuNumInRow = refPic->getWidth(COMPONENT_Y) / 64;
	//-----------------CMP 4x3-------------------//
	Int FaceNumInRow = 4;
	Int FaceNumInColumn = 3;
	Int PixelNumInRowInFace_O = refPic->getWidth(COMPONENT_Y) / FaceNumInRow;
	Int PixelNumInColumnInFace_O = refPic->getHeight(COMPONENT_Y) / FaceNumInColumn;
	Int PixelNumInRowInFace = PixelNumInRowInFace_O * 4;
	Int PixelNumInColumnInFace = PixelNumInColumnInFace_O * 4;
	Int N = PixelNumInRowInFace / 2;

	Int CtuX = CtuRsAddr%CtuNumInRow;
	Int CtuY = CtuRsAddr / CtuNumInRow;
	Int PuRsAddr = g_auiZscanToRaster[cu->getZorderIdxInCtu() + partAddr];
	Int PuNumInRow = 16;
	Int PuX = PuRsAddr%PuNumInRow;
	Int PuY = PuRsAddr / PuNumInRow;
	Int LAX = 0; Int LAY = 0;
	Int RAX = 0; Int RAY = 0;
	Int LBX = 0; Int LBY = 0;
	Int RBX = 0; Int RBY = 0;
	Int MCX = 0; Int MCY = 0;
	//---------------------calculate coordinates of current PU in the picture--------------//
	//-----multiple coordinates of current PU in the picture with 4 to keep the accuracy------------//
	Int CurrentPUX = 4 * (CtuX * 64 + PuX * 4);
	Int CurrentPUY = 4 * (CtuY * 64 + PuY * 4);

	//Int mvX = mv->getHor() >> shiftHor;
	//Int mvY = mv->getVer() >> shiftVer;
	Int mvX = mv->getHor();
	Int mvY = mv->getVer();
	//---------------------calculate coordinates of reference block in the picture--------------//
	Int ReferenceX = 0;
	Int ReferenceY = 0;
	double ReferenceXSub = 0;
	double ReferenceYSub = 0;//Sub_coordinate of the extension
	double ReferenceXSubPro = 0;
	double ReferenceYSubPro = 0;
	double ReferenceXProjection = 0;
	double ReferenceYProjection = 0;//侧面坐标系
	Int FaceNum = 8;
	Int TempCurrentPUX = CurrentPUX;
	Int TempCurrentPUY = CurrentPUY;
	Int width1 = 0;
	Int height1 = 0;
	Int width2 = 0;
	Int height2 = 0;
	//-------------------Sub-block Motion Compensation------------------//
	Int SBWidth = width / 4;
	Int SBHeight = height / 4;
	Int a = 2;

	ReferenceX = CurrentPUX + mvX;
	ReferenceY = CurrentPUY + mvY;


	if (CurrentPUX / PixelNumInRowInFace == 0)
	{
		if (CurrentPUY / PixelNumInColumnInFace == 0)
		{
			FaceNum = 1;
		}
		else if (CurrentPUY / PixelNumInColumnInFace == 1)
		{
			FaceNum = 2;
		}
		else if (CurrentPUY / PixelNumInColumnInFace == 2)
		{
			FaceNum = 6;
		}
	}
	else if (CurrentPUX / PixelNumInRowInFace == 1)
	{
		if (CurrentPUY / PixelNumInColumnInFace == 0)
		{
			FaceNum = 0;//FaceNum=0, current CU belongs to no-data grey area
		}
		else if (CurrentPUY / PixelNumInColumnInFace == 1)
		{
			FaceNum = 3;
		}
		else if (CurrentPUY / PixelNumInColumnInFace == 2)
		{
			FaceNum = 0;
		}
	}
	else if (CurrentPUX / PixelNumInRowInFace == 2)
	{
		if (CurrentPUY / PixelNumInColumnInFace == 0)
		{
			FaceNum = 0;
		}
		else if (CurrentPUY / PixelNumInColumnInFace == 1)
		{
			FaceNum = 4;
		}
		else if (CurrentPUY / PixelNumInColumnInFace == 2)
		{
			FaceNum = 0;
		}
	}
	else
	{
		if (CurrentPUY / PixelNumInColumnInFace == 0)
		{
			FaceNum = 0;
		}
		else if (CurrentPUY / PixelNumInColumnInFace == 1)
		{
			FaceNum = 5;
		}
		else if (CurrentPUY / PixelNumInColumnInFace == 2)
		{
			FaceNum = 0;
		}
	}
	//------------------regular block motion compensation or sub-block projection-based motion compensation------------------------------------------------//
	if ((ReferenceX + 12>0 && ReferenceY + 12>0) && (CurrentPUX / PixelNumInRowInFace == (ReferenceX + 12) / PixelNumInRowInFace || CurrentPUX / PixelNumInRowInFace == ReferenceX / PixelNumInRowInFace) && (CurrentPUY / PixelNumInColumnInFace == (ReferenceY + 12) / PixelNumInColumnInFace || CurrentPUY / PixelNumInColumnInFace == ReferenceY / PixelNumInColumnInFace) && ((ReferenceX + ((width - 1) * 4 - 12)>0 && ReferenceY + 4 * (height - 1) - 12>0) && CurrentPUX + (width - 1) * 4) / PixelNumInRowInFace == (ReferenceX + (width - 1) * 4 - 12) / PixelNumInRowInFace && (CurrentPUY + 4 * (height - 1)) / PixelNumInColumnInFace == (ReferenceY + 4 * (height - 1) - 12) / PixelNumInColumnInFace)
	{//-------------reference block and current CU belong to the same face,regular motion compensation----------------------------------------------------//
		xPredInterBlk(compID, cu, refPic, partAddr, mv, width, height, dstPic, bi, bitDepth);
	}//-------------reference block and current CU do not belong to the same face----------------------------------------------------//
	else if (((ReferenceX + 12>0 && ReferenceY + 12>0) && (CurrentPUX / PixelNumInRowInFace == (ReferenceX + 12) / PixelNumInRowInFace || CurrentPUX / PixelNumInRowInFace == ReferenceX / PixelNumInRowInFace) && (CurrentPUY / PixelNumInColumnInFace == (ReferenceY + 12) / PixelNumInColumnInFace || CurrentPUY / PixelNumInColumnInFace == ReferenceY / PixelNumInColumnInFace)) || ((CurrentPUX + (width - 1) * 4) / PixelNumInRowInFace == (ReferenceX + (width - 1) * 4 - 12) / PixelNumInRowInFace && (CurrentPUY + 4 * (height - 1)) / PixelNumInColumnInFace == (ReferenceY + 4 * (height - 1) - 12) / PixelNumInColumnInFace) && ((ReferenceX + ((width - 1) * 4 - 12)>0 && ReferenceY + 4 * (height - 1) - 12>0)))
	{
		if ((ReferenceX + 12>0 && ReferenceY + 12>0) && (CurrentPUX / PixelNumInRowInFace == (ReferenceX + 12) / PixelNumInRowInFace || CurrentPUX / PixelNumInRowInFace == ReferenceX / PixelNumInRowInFace) && (CurrentPUY / PixelNumInColumnInFace == (ReferenceY + 12) / PixelNumInColumnInFace || CurrentPUY / PixelNumInColumnInFace == ReferenceY / PixelNumInColumnInFace))
		{
			ReferenceX = ReferenceX + (width - 1) * 4;
			ReferenceY = ReferenceY + (height - 1) * 4;
			switch (FaceNum)
			{
			case 1:// current face belong to face 1
				//-------------------------------Get coordinates of four corners of face 1--------------------------------//
				LAX = 0; LAY = 0;
				RAX = 2 * N - 1; RAY = 0;
				LBX = 0; LBY = 2 * N - 1;
				RBX = 2 * N - 1; RBY = 2 * N - 1;
				MCX = (LAX + RAX + 1) / 2;
				MCY = (LAY + 1 + LBY) / 2;
				if (ReferenceX - RAX > 0 && abs(ReferenceX - MCX) > abs(ReferenceY - MCY)) //Area A
				{
					ReferenceXSub = ReferenceX - 2 * N;
					ReferenceYSub = N + 1 - ReferenceY;
					//---------------------------Projection Transform----------------------------------//
					ReferenceXSubPro = (N * ReferenceXSub) / (N + ReferenceXSub);
					ReferenceYSubPro = (N * ReferenceYSub) / (N + ReferenceXSub);
					//---------------------------Coordinates of Reference Block after modification in picture------------------//
					ReferenceXProjection = 3 * N + ReferenceYSubPro;
					ReferenceYProjection = 2 * N + ReferenceXSubPro - 1;
					RotationIdx = 1;
					//-------------------------calculate half-block size-----------------------//
					height1 = height;
					height2 = height;
					width2 = (abs(ReferenceX) % 3840 + 8) / 16 * 4;
					width1 = width - width2;
				}
				else if (ReferenceY - LAY < 0 && abs(ReferenceX - MCX) < abs(ReferenceY - MCY))  //Area B
				{
					ReferenceXSub = 1 - ReferenceY;
					ReferenceYSub = N + 1 - ReferenceX;
					//---------------------------Projection Transform----------------------------------//
					ReferenceXSubPro = (N * ReferenceXSub) / (N + ReferenceXSub);
					ReferenceYSubPro = (N * ReferenceYSub) / (N + ReferenceXSub);
					//---------------------------Coordinates of Reference Block after modification in picture------------------//
					ReferenceXProjection = 5 * N + ReferenceXSubPro - 1;
					ReferenceYProjection = 2 * N + ReferenceYSubPro - 1;
					RotationIdx = 2;
					//-------------------------calculate half-block size-----------------------//
					height1 = height;
					height2 = 0;
					width2 = 0;
					width1 = width;
				}
				else if (ReferenceX - LAX < 0 && abs(ReferenceX - MCX) > abs(ReferenceY - MCY))  //Area C
				{
					ReferenceXSub = 1 - ReferenceX;
					ReferenceYSub = ReferenceY - N;
					//---------------------------Projection Transform----------------------------------//
					ReferenceXSubPro = (N * ReferenceXSub) / (N + ReferenceXSub);
					ReferenceYSubPro = (N * ReferenceYSub) / (N + ReferenceXSub);
					//---------------------------Coordinates of Reference Block after modification in picture------------------//
					ReferenceXProjection = 7 * N + ReferenceYSubPro - 1;
					ReferenceYProjection = 2 * N + ReferenceXSubPro;
					RotationIdx = 3;
					//-------------------------calculate half-block size-----------------------//
					height1 = height;
					height2 = 0;
					width2 = 0;
					width1 = width;
				}
				else if (ReferenceY - LBY > 0 && abs(ReferenceX - MCX) < abs(ReferenceY - MCY)) //Area D
				{
					ReferenceXSub = ReferenceY - 2 * N;
					ReferenceYSub = ReferenceX - N;
					//---------------------------Projection Transform----------------------------------//
					ReferenceXSubPro = (N * ReferenceXSub) / (N + ReferenceXSub);
					ReferenceYSubPro = (N * ReferenceYSub) / (N + ReferenceXSub);
					//---------------------------Coordinates of Reference Block after modification in picture------------------//
					ReferenceXProjection = N + ReferenceYSubPro;
					ReferenceYProjection = 2 * N + ReferenceXSubPro;
					RotationIdx = 0;
					//-------------------------calculate half-block size-----------------------//
					width2 = width;
					width1 = width;
					height2 = (abs(ReferenceY) % 3840 + 8) / 16 * 4;
					height1 = height - height2;
				}
				else
				{
					ReferenceXProjection = ReferenceX;
					ReferenceYProjection = ReferenceY;
					RotationIdx = 0;
					height1 = height;
					height2 = 0;
					width2 = 0;
					width1 = width;
				}
				break;

			case 2:// current face belong to face 2
				//-------------------------------Get coordinates of four corners of face 2--------------------------------//
				LAX = 0; LAY = 2 * N;
				RAX = 2 * N - 1; RAY = 2 * N;
				LBX = 0; LBY = 4 * N - 1;
				RBX = 2 * N - 1; RBY = 4 * N - 1;
				MCX = (LAX + RAX + 1) / 2;
				MCY = (LAY + 1 + LBY) / 2;
				if (ReferenceX - RAX > 0 && abs(ReferenceX - MCX) > abs(ReferenceY - MCY)) //Area A
				{
					ReferenceXSub = ReferenceX - 2 * N;
					ReferenceYSub = 3 * N + 1 - ReferenceY;
					//---------------------------Projection Transform----------------------------------//
					ReferenceXSubPro = (N * ReferenceXSub) / (N + ReferenceXSub);
					ReferenceYSubPro = (N * ReferenceYSub) / (N + ReferenceXSub);
					//---------------------------Coordinates of Reference Block after modification in picture------------------//
					ReferenceXProjection = 2 * N + ReferenceXSubPro;
					ReferenceYProjection = 3 * N - ReferenceYSubPro - 1;
					RotationIdx = 0;
					//-------------------------calculate half-block size-----------------------//
					height1 = height;
					height2 = height;
					width2 = (abs(ReferenceX) % 3840 + 8) / 16 * 4;
					width1 = width - width2;
				}
				else if (ReferenceY - LAY < 0 && abs(ReferenceX - MCX) < abs(ReferenceY - MCY))  //Area B
				{
					ReferenceXSub = 2 * N + 1 - ReferenceY;
					ReferenceYSub = N + 1 - ReferenceX;
					//---------------------------Projection Transform----------------------------------//
					ReferenceXSubPro = (N * ReferenceXSub) / (N + ReferenceXSub);
					ReferenceYSubPro = (N * ReferenceYSub) / (N + ReferenceXSub);
					//---------------------------Coordinates of Reference Block after modification in picture------------------//
					ReferenceXProjection = N - ReferenceYSubPro - 1;
					ReferenceYProjection = 2 * N - ReferenceXSubPro - 1;
					RotationIdx = 0;
					//-------------------------calculate half-block size-----------------------//
					height1 = height;
					height2 = 0;
					width2 = 0;
					width1 = width;
				}
				else if (ReferenceX - LAX < 0 && abs(ReferenceX - MCX) > abs(ReferenceY - MCY))  //Area C
				{
					ReferenceXSub = 1 - ReferenceX;
					ReferenceYSub = ReferenceY - 3 * N;
					//---------------------------Projection Transform----------------------------------//
					ReferenceXSubPro = (N * ReferenceXSub) / (N + ReferenceXSub);
					ReferenceYSubPro = (N * ReferenceYSub) / (N + ReferenceXSub);
					//---------------------------Coordinates of Reference Block after modification in picture------------------//
					ReferenceXProjection = 8 * N - ReferenceXSubPro - 1;
					ReferenceYProjection = 3 * N + ReferenceYSubPro;
					RotationIdx = 0;
					//-------------------------calculate half-block size-----------------------//
					height1 = height;
					height2 = 0;
					width2 = 0;
					width1 = width;
				}
				else if (ReferenceY - LBY > 0 && abs(ReferenceX - MCX) < abs(ReferenceY - MCY)) //Area D
				{
					ReferenceXSub = ReferenceY - 4 * N;
					ReferenceYSub = ReferenceX - N;
					//---------------------------Projection Transform----------------------------------//
					ReferenceXSubPro = (N * ReferenceXSub) / (N + ReferenceXSub);
					ReferenceYSubPro = (N * ReferenceYSub) / (N + ReferenceXSub);
					//---------------------------Coordinates of Reference Block after modification in picture------------------//
					ReferenceXProjection = N + ReferenceYSubPro;
					ReferenceYProjection = 2 * N + ReferenceXSubPro;
					RotationIdx = 0;
					//-------------------------calculate half-block size-----------------------//
					width2 = width;
					width1 = width;
					height2 = (abs(ReferenceY) % 3840 + 8) / 16 * 4;
					height1 = height - height2;
				}
				else
				{
					ReferenceXProjection = ReferenceX;
					ReferenceYProjection = ReferenceY;
					RotationIdx = 0;
					height1 = height;
					height2 = 0;
					width2 = 0;
					width1 = width;
				}
				break;

			case 3:// current face belong to face 3
				//-------------------------------Get coordinates of four corners of face 3--------------------------------//
				LAX = 2 * N; LAY = 2 * N;
				RAX = 4 * N - 1; RAY = 2 * N;
				LBX = 2 * N; LBY = 4 * N - 1;
				RBX = 4 * N - 1; RBY = 4 * N - 1;
				MCX = (LAX + RAX + 1) / 2;
				MCY = (LAY + 1 + LBY) / 2;
				if (ReferenceX - RAX > 0 && abs(ReferenceX - MCX) > abs(ReferenceY - MCY)) //Area A
				{
					ReferenceXSub = ReferenceX - 4 * N;
					ReferenceYSub = 3 * N + 1 - ReferenceY;
					//---------------------------Projection Transform----------------------------------//
					ReferenceXSubPro = (N * ReferenceXSub) / (N + ReferenceXSub);
					ReferenceYSubPro = (N * ReferenceYSub) / (N + ReferenceXSub);
					//---------------------------Coordinates of Reference Block after modification in picture------------------//
					ReferenceXProjection = 4 * N + ReferenceXSubPro;
					ReferenceYProjection = 3 * N + ReferenceYSubPro - 1;
					RotationIdx = 0;
					//-------------------------calculate half-block size-----------------------//
					height1 = height;
					height2 = height;
					width2 = (abs(ReferenceX) % 3840 + 8) / 16 * 4;
					width1 = width - width2;
				}
				else if (ReferenceY - LAY < 0 && abs(ReferenceX - MCX) < abs(ReferenceY - MCY))  //Area B
				{
					ReferenceXSub = 2 * N + 1 - ReferenceY;
					ReferenceYSub = 3 * N + 1 - ReferenceX;
					//---------------------------Projection Transform----------------------------------//
					ReferenceXSubPro = (N * ReferenceXSub) / (N + ReferenceXSub);
					ReferenceYSubPro = (N * ReferenceYSub) / (N + ReferenceXSub);
					//---------------------------Coordinates of Reference Block after modification in picture------------------//
					ReferenceXProjection = 2 * N - ReferenceXSubPro - 1;
					ReferenceYProjection = N + ReferenceYSubPro - 1;
					RotationIdx = 3;
					//-------------------------calculate half-block size-----------------------//
					height1 = height;
					height2 = 0;
					width2 = 0;
					width1 = width;
				}
				else if (ReferenceX - LAX < 0 && abs(ReferenceX - MCX) > abs(ReferenceY - MCY))  //Area C
				{
					ReferenceXSub = 2 * N + 1 - ReferenceX;
					ReferenceYSub = ReferenceY - 3 * N;
					//---------------------------Projection Transform----------------------------------//
					ReferenceXSubPro = (N * ReferenceXSub) / (N + ReferenceXSub);
					ReferenceYSubPro = (N * ReferenceYSub) / (N + ReferenceXSub);
					//---------------------------Coordinates of Reference Block after modification in picture------------------//
					ReferenceXProjection = 2 * N - ReferenceXSubPro - 1;
					ReferenceYProjection = 3 * N + ReferenceYSubPro;
					RotationIdx = 0;
					//-------------------------calculate half-block size-----------------------//
					height1 = height;
					height2 = 0;
					width2 = 0;
					width1 = width;
				}
				else if (ReferenceY - LBY > 0 && abs(ReferenceX - MCX) < abs(ReferenceY - MCY)) //Area D

				{
					ReferenceXSub = ReferenceY - 4 * N;
					ReferenceYSub = ReferenceX - 3 * N;
					//---------------------------Projection Transform----------------------------------//
					ReferenceXSubPro = (N * ReferenceXSub) / (N + ReferenceXSub);
					ReferenceYSubPro = (N * ReferenceYSub) / (N + ReferenceXSub);
					//---------------------------Coordinates of Reference Block after modification in picture------------------//
					ReferenceXProjection = 2 * N - ReferenceXSubPro;
					ReferenceYProjection = 5 * N + ReferenceYSubPro;
					RotationIdx = 1;
					//-------------------------calculate half-block size-----------------------//
					width2 = width;
					width1 = width;
					height2 = (abs(ReferenceY) % 3840 + 8) / 16 * 4;
					height1 = height - height2;
				}
				else
				{
					ReferenceXProjection = ReferenceX;
					ReferenceYProjection = ReferenceY;
					RotationIdx = 0;
					height1 = height;
					height2 = 0;
					width2 = 0;
					width1 = width;
				}
				break;

			case 4:// current face belong to face 4
				//-------------------------------Get coordinates of four corners of face 4--------------------------------//
				LAX = 4 * N; LAY = 2 * N;
				RAX = 6 * N - 1; RAY = 2 * N;
				LBX = 4 * N; LBY = 4 * N - 1;
				RBX = 6 * N - 1; RBY = 4 * N - 1;
				MCX = (LAX + RAX + 1) / 2;
				MCY = (LAY + 1 + LBY) / 2;
				if (ReferenceX - RAX > 0 && abs(ReferenceX - MCX) > abs(ReferenceY - MCY)) //Area A
				{
					ReferenceXSub = ReferenceX - 6 * N;
					ReferenceYSub = 3 * N + 1 - ReferenceY;
					//---------------------------Projection Transform----------------------------------//
					ReferenceXSubPro = (N * ReferenceXSub) / (N + ReferenceXSub);
					ReferenceYSubPro = (N * ReferenceYSub) / (N + ReferenceXSub);
					//---------------------------Coordinates of Reference Block after modification in picture------------------//
					ReferenceXProjection = 6 * N + ReferenceXSubPro;
					ReferenceYProjection = 3 * N + ReferenceYSubPro - 1;
					RotationIdx = 0;
					//-------------------------calculate half-block size-----------------------//
					height1 = height;
					height2 = height;
					width2 = (abs(ReferenceX) % 3840 + 8) / 16 * 4;
					width1 = width - width2;
				}
				else if (ReferenceY - LAY < 0 && abs(ReferenceX - MCX) < abs(ReferenceY - MCY))  //Area B
				{
					ReferenceXSub = 2 * N + 1 - ReferenceY;
					ReferenceYSub = 5 * N + 1 - ReferenceX;
					//---------------------------Projection Transform----------------------------------//
					ReferenceXSubPro = (N * ReferenceXSub) / (N + ReferenceXSub);
					ReferenceYSubPro = (N * ReferenceYSub) / (N + ReferenceXSub);
					//---------------------------Coordinates of Reference Block after modification in picture------------------//
					ReferenceXProjection = N + ReferenceYSubPro - 1;
					ReferenceYProjection = ReferenceXSubPro - 1;
					RotationIdx = 2;
					//-------------------------calculate half-block size-----------------------//
					height1 = height;
					height2 = 0;
					width2 = 0;
					width1 = width;
				}
				else if (ReferenceX - LAX < 0 && abs(ReferenceX - MCX) > abs(ReferenceY - MCY))  //Area C
				{
					ReferenceXSub = 4 * N + 1 - ReferenceX;
					ReferenceYSub = ReferenceY - 3 * N;
					//---------------------------Projection Transform----------------------------------//
					ReferenceXSubPro = (N * ReferenceXSub) / (N + ReferenceXSub);
					ReferenceYSubPro = (N * ReferenceYSub) / (N + ReferenceXSub);
					//---------------------------Coordinates of Reference Block after modification in picture------------------//
					ReferenceXProjection = 4 * N - ReferenceXSubPro - 1;
					ReferenceYProjection = 3 * N + ReferenceYSubPro;
					RotationIdx = 0;
					//-------------------------calculate half-block size-----------------------//
					height1 = height;
					height2 = 0;
					width2 = 0;
					width1 = width;
				}
				else if (ReferenceY - LBY > 0 && abs(ReferenceX - MCX) < abs(ReferenceY - MCY)) //Area D
				{
					ReferenceXSub = ReferenceY - 4 * N;
					ReferenceYSub = ReferenceX - 5 * N;
					//---------------------------Projection Transform----------------------------------//
					ReferenceXSubPro = (N * ReferenceXSub) / (N + ReferenceXSub);
					ReferenceYSubPro = (N * ReferenceYSub) / (N + ReferenceXSub);
					//--------------------------Coordinates of Reference Block after modification in picture------------------//
					ReferenceXProjection = N - ReferenceYSubPro;
					ReferenceYProjection = 6 * N - ReferenceXSubPro;
					RotationIdx = 2;
					//-------------------------calculate half-block size-----------------------//
					width2 = width;
					width1 = width;
					height2 = (abs(ReferenceY) % 3840 + 8) / 16 * 4;
					height1 = height - height2;
				}
				else
				{
					ReferenceXProjection = ReferenceX;
					ReferenceYProjection = ReferenceY;
					RotationIdx = 0;
					height1 = height;
					height2 = 0;
					width2 = 0;
					width1 = width;
				}
				break;

			case 5:// current face belong to face 5
				//-------------------------------Get coordinates of four corners of face 5--------------------------------//
				LAX = 6 * N; LAY = 2 * N;
				RAX = 8 * N - 1; RAY = 2 * N;
				LBX = 6 * N; LBY = 4 * N - 1;
				RBX = 8 * N - 1; RBY = 4 * N - 1;
				MCX = (LAX + RAX + 1) / 2;
				MCY = (LAY + 1 + LBY) / 2;
				if (ReferenceX - RAX > 0 && abs(ReferenceX - MCX) > abs(ReferenceY - MCY)) //Area A
				{
					ReferenceXSub = ReferenceX - 8 * N;
					ReferenceYSub = 3 * N + 1 - ReferenceY;
					//---------------------------Projection Transform----------------------------------//
					ReferenceXSubPro = (N * ReferenceXSub) / (N + ReferenceXSub);
					ReferenceYSubPro = (N * ReferenceYSub) / (N + ReferenceXSub);
					//---------------------------Coordinates of Reference Block after modification in picture------------------//
					ReferenceXProjection = ReferenceXSubPro;
					ReferenceYProjection = 3 * N + ReferenceYSubPro - 1;
					RotationIdx = 0;
					//-------------------------calculate half-block size-----------------------//
					height1 = height;
					height2 = height;
					width2 = (abs(ReferenceX) % 3840 + 8) / 16 * 4;
					width1 = width - width2;
				}
				else if (ReferenceY - LAY < 0 && abs(ReferenceX - MCX) < abs(ReferenceY - MCY))  //Area B
				{
					ReferenceXSub = 2 * N + 1 - ReferenceY;
					ReferenceYSub = 7 * N + 1 - ReferenceX;
					//---------------------------Projection Transform----------------------------------//
					ReferenceXSubPro = (N * ReferenceXSub) / (N + ReferenceXSub);
					ReferenceYSubPro = (N * ReferenceYSub) / (N + ReferenceXSub);
					//---------------------------Coordinates of Reference Block after modification in picture------------------//
					ReferenceXProjection = ReferenceXSubPro - 1;
					ReferenceYProjection = N - ReferenceYSubPro - 1;
					RotationIdx = 1;
					//-------------------------calculate half-block size-----------------------//
					height1 = height;
					height2 = 0;
					width2 = 0;
					width1 = width;
				}
				else if (ReferenceX - LAX < 0 && abs(ReferenceX - MCX) > abs(ReferenceY - MCY))  //Area C
				{
					ReferenceXSub = 6 * N + 1 - ReferenceX;
					ReferenceYSub = ReferenceY - 3 * N;
					//---------------------------Projection Transform----------------------------------//
					ReferenceXSubPro = (N * ReferenceXSub) / (N + ReferenceXSub);
					ReferenceYSubPro = (N * ReferenceYSub) / (N + ReferenceXSub);
					//---------------------------Coordinates of Reference Block after modification in picture------------------//
					ReferenceXProjection = 6 * N - ReferenceXSubPro - 1;
					ReferenceYProjection = 3 * N + ReferenceYSubPro;
					RotationIdx = 0;
					//-------------------------calculate half-block size-----------------------//
					height1 = height;
					height2 = 0;
					width2 = 0;
					width1 = width;
				}
				else if (ReferenceY - LBY > 0 && abs(ReferenceX - MCX) < abs(ReferenceY - MCY)) //Area D
				{
					ReferenceXSub = ReferenceY - 4 * N;
					ReferenceYSub = ReferenceX - 7 * N;
					//---------------------------Projection Transform----------------------------------//
					ReferenceXSubPro = (N * ReferenceXSub) / (N + ReferenceXSub);
					ReferenceYSubPro = (N * ReferenceYSub) / (N + ReferenceXSub);
					//---------------------------Coordinates of Reference Block after modification in picture------------------//
					ReferenceXProjection = ReferenceXSubPro;
					ReferenceYProjection = 5 * N - ReferenceYSubPro;
					RotationIdx = 3;
					//-------------------------calculate half-block size-----------------------//
					width2 = width;
					width1 = width;
					height2 = (abs(ReferenceY) % 3840 + 8) / 16 * 4;
					height1 = height - height2;
				}
				else
				{
					ReferenceXProjection = ReferenceX;
					ReferenceYProjection = ReferenceY;
					RotationIdx = 0;
					height1 = height;
					height2 = 0;
					width2 = 0;
					width1 = width;
				}
				break;

			case 6:// current face belong to face 6
				//-------------------------------Get coordinates of four corners of face 6--------------------------------//
				LAX = 0; LAY = 4 * N;
				RAX = 2 * N - 1; RAY = 4 * N;
				LBX = 0; LBY = 6 * N - 1;
				RBX = 2 * N - 1; RBY = 6 * N - 1;
				MCX = (LAX + RAX + 1) / 2;
				MCY = (LAY + 1 + LBY) / 2;
				if (ReferenceX - RAX > 0 && abs(ReferenceX - MCX) > abs(ReferenceY - MCY)) //Area A
				{
					ReferenceXSub = ReferenceX - 2 * N;
					ReferenceYSub = 5 * N + 1 - ReferenceY;
					//---------------------------Projection Transform----------------------------------//
					ReferenceXSubPro = (N * ReferenceXSub) / (N + ReferenceXSub);
					ReferenceYSubPro = (N * ReferenceYSub) / (N + ReferenceXSub);
					//---------------------------Coordinates of Reference Block after modification in picture------------------//
					ReferenceXProjection = 3 * N - ReferenceYSubPro;
					ReferenceYProjection = 4 * N - ReferenceXSubPro - 1;
					RotationIdx = 3;
					//-------------------------calculate half-block size-----------------------//
					height1 = height;
					height2 = height;
					width2 = (abs(ReferenceX) % 3840 + 8) / 16 * 4;
					width1 = width - width2;
				}
				else if (ReferenceY - LAY < 0 && abs(ReferenceX - MCX) < abs(ReferenceY - MCY))  //Area B
				{
					ReferenceXSub = 4 * N + 1 - ReferenceY;
					ReferenceYSub = N + 1 - ReferenceX;
					//---------------------------Projection Transform----------------------------------//
					ReferenceXSubPro = (N * ReferenceXSub) / (N + ReferenceXSub);
					ReferenceYSubPro = (N * ReferenceYSub) / (N + ReferenceXSub);
					//---------------------------Coordinates of Reference Block after modification in picture------------------//
					ReferenceXProjection = N - ReferenceYSubPro - 1;
					ReferenceYProjection = 4 * N - ReferenceXSubPro - 1;
					RotationIdx = 0;
					//-------------------------calculate half-block size-----------------------//
					height1 = height;
					height2 = 0;
					width2 = 0;
					width1 = width;
				}
				else if (ReferenceX - LAX < 0 && abs(ReferenceX - MCX) > abs(ReferenceY - MCY))  //Area C
				{
					ReferenceXSub = 1 - ReferenceX;
					ReferenceYSub = ReferenceY - 5 * N;
					//---------------------------Projection Transform----------------------------------//
					ReferenceXSubPro = (N * ReferenceXSub) / (N + ReferenceXSub);
					ReferenceYSubPro = (N * ReferenceYSub) / (N + ReferenceXSub);
					//---------------------------Coordinates of Reference Block after modification in picture------------------//
					ReferenceXProjection = 7 * N - ReferenceYSubPro - 1;
					ReferenceYProjection = 4 * N - ReferenceXSubPro;
					RotationIdx = 1;
					//-------------------------calculate half-block size-----------------------//
					height1 = height;
					height2 = 0;
					width2 = 0;
					width1 = width;
				}
				else if (ReferenceY - LBY > 0 && abs(ReferenceX - MCX) < abs(ReferenceY - MCY)) //Area D
				{
					ReferenceXSub = ReferenceY - 6 * N;
					ReferenceYSub = ReferenceX - N;
					//---------------------------Projection Transform----------------------------------//
					ReferenceXSubPro = (N * ReferenceXSub) / (N + ReferenceXSub);
					ReferenceYSubPro = (N * ReferenceYSub) / (N + ReferenceXSub);
					//---------------------------Coordinates of Reference Block after modification in picture------------------//
					ReferenceXProjection = 5 * N - ReferenceYSubPro;
					ReferenceYProjection = 4 * N - ReferenceXSubPro;
					RotationIdx = 2;
					//-------------------------calculate half-block size-----------------------//
					width2 = width;
					width1 = width;
					height2 = (abs(ReferenceY) % 3840 + 8) / 16 * 4;
					height1 = height - height2;
				}
				else
				{
					ReferenceXProjection = ReferenceX;
					ReferenceYProjection = ReferenceY;
					RotationIdx = 0;
					height1 = height;
					height2 = 0;
					width2 = 0;
					width1 = width;
				}
				break;
			case 0:
				ReferenceXProjection = ReferenceX;
				ReferenceYProjection = ReferenceY;
				RotationIdx = 0;
				height1 = height;
				height2 = 0;
				width2 = 0;
				width1 = width;
				break;
			case 8:

				cerr << "FaceNum not assigned.2/n" << endl;

				break;
			default:

				cerr << "FaceNum = " << FaceNum << "/n" << endl;

				break;
			}
			Int mvX_2 = ReferenceXProjection - CurrentPUX - (width - 1) * 4;
			Int mvY_2 = ReferenceYProjection - CurrentPUY - (height - 1) * 4;
			TComMv mv1(0, 0);
			mv1.setHor(mvX_2);
			mv1.setVer(mvY_2);

			
			

		
			assert(height1*width1 + width2*height2 == width*height);
			assert(height1 >= 0 && width1 >= 0 && height2 >= 0 && width2 >= 0);
		

			{

				// if(height1*width1)
				xPredInterBlk(compID, cu, refPic, partAddr, mv, width1, height1, dstPic, bi, bitDepth);
				//if(height2*width2)
				xPredInterBlk_PBSMC(compID, cu, refPic, g_auiRasterToZscan[g_auiZscanToRaster[partAddr] + (width1%width) / 4 + (height1%height) * 4], mv1, width2, height2, dstPic, bi, bitDepth, RotationIdx);

			}
		}
		else
		{
			switch (FaceNum)
			{
			case 1:// current face belong to face 1
				//-------------------------------Get coordinates of four corners of face 1--------------------------------//
				LAX = 0; LAY = 0;
				RAX = 2 * N - 1; RAY = 0;
				LBX = 0; LBY = 2 * N - 1;
				RBX = 2 * N - 1; RBY = 2 * N - 1;
				MCX = (LAX + RAX + 1) / 2;
				MCY = (LAY + 1 + LBY) / 2;
				if (ReferenceX - RAX > 0 && abs(ReferenceX - MCX) > abs(ReferenceY - MCY)) //Area A
				{
					ReferenceXSub = ReferenceX - 2 * N;
					ReferenceYSub = N + 1 - ReferenceY;
					//---------------------------Projection Transform----------------------------------//
					ReferenceXSubPro = (N * ReferenceXSub) / (N + ReferenceXSub);
					ReferenceYSubPro = (N * ReferenceYSub) / (N + ReferenceXSub);
					//---------------------------Coordinates of Reference Block after modification in picture------------------//
					ReferenceXProjection = 3 * N + ReferenceYSubPro;
					ReferenceYProjection = 2 * N + ReferenceXSubPro - 1;
					RotationIdx = 1;
					//-------------------------calculate half-block size-----------------------//
					height1 = height;
					height2 = 0;
					width2 = 0;
					width1 = width;
				}
				else if (ReferenceY - LAY < 0 && abs(ReferenceX - MCX) < abs(ReferenceY - MCY))  //Area B
				{
					ReferenceXSub = 1 - ReferenceY;
					ReferenceYSub = N + 1 - ReferenceX;
					//---------------------------Projection Transform----------------------------------//
					ReferenceXSubPro = (N * ReferenceXSub) / (N + ReferenceXSub);
					ReferenceYSubPro = (N * ReferenceYSub) / (N + ReferenceXSub);
					//---------------------------Coordinates of Reference Block after modification in picture------------------//
					ReferenceXProjection = 5 * N + ReferenceXSubPro - 1;
					ReferenceYProjection = 2 * N + ReferenceYSubPro - 1;
					RotationIdx = 2;
					//-------------------------calculate half-block size-----------------------//
					width2 = width;
					width1 = width;
					if (ReferenceY>0)
					{
						height2 = (3840 - ReferenceY % 3840 + 8) / 16 * 4;
					}
					else
					{
						height2 = (abs(ReferenceY) % 3840 + 8) / 16 * 4;
					}

					height1 = height - height2;
				}
				else if (ReferenceX - LAX < 0 && abs(ReferenceX - MCX) > abs(ReferenceY - MCY))  //Area C
				{
					ReferenceXSub = 1 - ReferenceX;
					ReferenceYSub = ReferenceY - N;
					//---------------------------Projection Transform----------------------------------//
					ReferenceXSubPro = (N * ReferenceXSub) / (N + ReferenceXSub);
					ReferenceYSubPro = (N * ReferenceYSub) / (N + ReferenceXSub);
					//---------------------------Coordinates of Reference Block after modification in picture------------------//
					ReferenceXProjection = 7 * N + ReferenceYSubPro - 1;
					ReferenceYProjection = 2 * N + ReferenceXSubPro;
					RotationIdx = 3;
					//-------------------------calculate half-block size-----------------------//
					height1 = height;
					height2 = height;
					if (ReferenceX>0)
					{
						width2 = (3840 - ReferenceX % 3840 + 8) / 16 * 4;
					}
					else
					{
						width2 = (abs(ReferenceX) % 3840 + 8) / 16 * 4;
					}
					width1 = width - width2;
				}
				else if (ReferenceY - LBY > 0 && abs(ReferenceX - MCX) < abs(ReferenceY - MCY)) //Area D
				{
					ReferenceXSub = ReferenceY - 2 * N;
					ReferenceYSub = ReferenceX - N;
					//---------------------------Projection Transform----------------------------------//
					ReferenceXSubPro = (N * ReferenceXSub) / (N + ReferenceXSub);
					ReferenceYSubPro = (N * ReferenceYSub) / (N + ReferenceXSub);
					//---------------------------Coordinates of Reference Block after modification in picture------------------//
					ReferenceXProjection = N + ReferenceYSubPro;
					ReferenceYProjection = 2 * N + ReferenceXSubPro;
					RotationIdx = 0;
					//-------------------------calculate half-block size-----------------------//
					height1 = height;
					height2 = 0;
					width2 = 0;
					width1 = width;
				}
				else
				{
					ReferenceXProjection = ReferenceX;
					ReferenceYProjection = ReferenceY;
					RotationIdx = 0;
					height1 = height;
					height2 = 0;
					width2 = 0;
					width1 = width;
				}
				break;

			case 2:// current face belong to face 2
				//-------------------------------Get coordinates of four corners of face 2--------------------------------//
				LAX = 0; LAY = 2 * N;
				RAX = 2 * N - 1; RAY = 2 * N;
				LBX = 0; LBY = 4 * N - 1;
				RBX = 2 * N - 1; RBY = 4 * N - 1;
				MCX = (LAX + RAX + 1) / 2;
				MCY = (LAY + 1 + LBY) / 2;
				if (ReferenceX - RAX > 0 && abs(ReferenceX - MCX) > abs(ReferenceY - MCY)) //Area A
				{
					ReferenceXSub = ReferenceX - 2 * N;
					ReferenceYSub = 3 * N + 1 - ReferenceY;
					//---------------------------Projection Transform----------------------------------//
					ReferenceXSubPro = (N * ReferenceXSub) / (N + ReferenceXSub);
					ReferenceYSubPro = (N * ReferenceYSub) / (N + ReferenceXSub);
					//---------------------------Coordinates of Reference Block after modification in picture------------------//
					ReferenceXProjection = 2 * N + ReferenceXSubPro;
					ReferenceYProjection = 3 * N - ReferenceYSubPro - 1;
					RotationIdx = 0;
					//-------------------------calculate half-block size-----------------------//
					height1 = height;
					height2 = 0;
					width2 = 0;
					width1 = width;
				}
				else if (ReferenceY - LAY < 0 && abs(ReferenceX - MCX) < abs(ReferenceY - MCY))  //Area B
				{
					ReferenceXSub = 2 * N + 1 - ReferenceY;
					ReferenceYSub = N + 1 - ReferenceX;
					//---------------------------Projection Transform----------------------------------//
					ReferenceXSubPro = (N * ReferenceXSub) / (N + ReferenceXSub);
					ReferenceYSubPro = (N * ReferenceYSub) / (N + ReferenceXSub);
					//---------------------------Coordinates of Reference Block after modification in picture------------------//
					ReferenceXProjection = N - ReferenceYSubPro - 1;
					ReferenceYProjection = 2 * N - ReferenceXSubPro - 1;
					RotationIdx = 0;
					//-------------------------calculate half-block size-----------------------//
					width2 = width;
					width1 = width;
					if (ReferenceY>0)
					{
						height2 = (3840 - ReferenceY % 3840 + 8) / 16 * 4;
					}
					else
					{
						height2 = (abs(ReferenceY) % 3840 + 8) / 16 * 4;
					}
					height1 = height - height2;
				}
				else if (ReferenceX - LAX < 0 && abs(ReferenceX - MCX) > abs(ReferenceY - MCY))  //Area C
				{
					ReferenceXSub = 1 - ReferenceX;
					ReferenceYSub = ReferenceY - 3 * N;
					//---------------------------Projection Transform----------------------------------//
					ReferenceXSubPro = (N * ReferenceXSub) / (N + ReferenceXSub);
					ReferenceYSubPro = (N * ReferenceYSub) / (N + ReferenceXSub);
					//---------------------------Coordinates of Reference Block after modification in picture------------------//
					ReferenceXProjection = 8 * N - ReferenceXSubPro - 1;
					ReferenceYProjection = 3 * N + ReferenceYSubPro;
					RotationIdx = 0;
					//-------------------------calculate half-block size-----------------------//
					height1 = height;
					height2 = height;
					if (ReferenceX>0)
					{
						width2 = (3840 - ReferenceX % 3840 + 8) / 16 * 4;
					}
					else
					{
						width2 = (abs(ReferenceX) % 3840 + 8) / 16 * 4;
					}
					width1 = width - width2;
				}
				else if (ReferenceY - LBY > 0 && abs(ReferenceX - MCX) < abs(ReferenceY - MCY)) //Area D
				{
					ReferenceXSub = ReferenceY - 4 * N;
					ReferenceYSub = ReferenceX - N;
					//---------------------------Projection Transform----------------------------------//
					ReferenceXSubPro = (N * ReferenceXSub) / (N + ReferenceXSub);
					ReferenceYSubPro = (N * ReferenceYSub) / (N + ReferenceXSub);
					//---------------------------Coordinates of Reference Block after modification in picture------------------//
					ReferenceXProjection = N + ReferenceYSubPro;
					ReferenceYProjection = 2 * N + ReferenceXSubPro;
					RotationIdx = 0;
					//-------------------------calculate half-block size-----------------------//
					height1 = height;
					height2 = 0;
					width2 = 0;
					width1 = width;
				}
				else
				{
					ReferenceXProjection = ReferenceX;
					ReferenceYProjection = ReferenceY;
					RotationIdx = 0;
					height1 = height;
					height2 = 0;
					width2 = 0;
					width1 = width;
				}
				break;

			case 3:// current face belong to face 3
				//-------------------------------Get coordinates of four corners of face 3--------------------------------//
				LAX = 2 * N; LAY = 2 * N;
				RAX = 4 * N - 1; RAY = 2 * N;
				LBX = 2 * N; LBY = 4 * N - 1;
				RBX = 4 * N - 1; RBY = 4 * N - 1;
				MCX = (LAX + RAX + 1) / 2;
				MCY = (LAY + 1 + LBY) / 2;
				if (ReferenceX - RAX > 0 && abs(ReferenceX - MCX) > abs(ReferenceY - MCY)) //Area A
				{
					ReferenceXSub = ReferenceX - 4 * N;
					ReferenceYSub = 3 * N + 1 - ReferenceY;
					//---------------------------Projection Transform----------------------------------//
					ReferenceXSubPro = (N * ReferenceXSub) / (N + ReferenceXSub);
					ReferenceYSubPro = (N * ReferenceYSub) / (N + ReferenceXSub);
					//---------------------------Coordinates of Reference Block after modification in picture------------------//
					ReferenceXProjection = 4 * N + ReferenceXSubPro;
					ReferenceYProjection = 3 * N + ReferenceYSubPro - 1;
					RotationIdx = 0;
					//-------------------------calculate half-block size-----------------------//
					height1 = height;
					height2 = 0;
					width2 = 0;
					width1 = width;
				}
				else if (ReferenceY - LAY < 0 && abs(ReferenceX - MCX) < abs(ReferenceY - MCY))  //Area B
				{
					ReferenceXSub = 2 * N + 1 - ReferenceY;
					ReferenceYSub = 3 * N + 1 - ReferenceX;
					//---------------------------Projection Transform----------------------------------//
					ReferenceXSubPro = (N * ReferenceXSub) / (N + ReferenceXSub);
					ReferenceYSubPro = (N * ReferenceYSub) / (N + ReferenceXSub);
					//---------------------------Coordinates of Reference Block after modification in picture------------------//
					ReferenceXProjection = 2 * N - ReferenceXSubPro - 1;
					ReferenceYProjection = N + ReferenceYSubPro - 1;
					RotationIdx = 3;
					//-------------------------calculate half-block size-----------------------//
					width2 = width;
					width1 = width;
					if (ReferenceY>0)
					{
						height2 = (3840 - ReferenceY % 3840 + 8) / 16 * 4;
					}
					else
					{
						height2 = (abs(ReferenceY) % 3840 + 8) / 16 * 4;
					}
					height1 = height - height2;
				}
				else if (ReferenceX - LAX < 0 && abs(ReferenceX - MCX) > abs(ReferenceY - MCY))  //Area C
				{
					ReferenceXSub = 2 * N + 1 - ReferenceX;
					ReferenceYSub = ReferenceY - 3 * N;
					//---------------------------Projection Transform----------------------------------//
					ReferenceXSubPro = (N * ReferenceXSub) / (N + ReferenceXSub);
					ReferenceYSubPro = (N * ReferenceYSub) / (N + ReferenceXSub);
					//---------------------------Coordinates of Reference Block after modification in picture------------------//
					ReferenceXProjection = 2 * N - ReferenceXSubPro - 1;
					ReferenceYProjection = 3 * N + ReferenceYSubPro;
					RotationIdx = 0;
					//-------------------------calculate half-block size-----------------------//
					height1 = height;
					height2 = height;
					if (ReferenceX>0)
					{
						width2 = (3840 - ReferenceX % 3840 + 8) / 16 * 4;
					}
					else
					{
						width2 = (abs(ReferenceX) % 3840 + 8) / 16 * 4;
					}
					width1 = width - width2;
				}
				else if (ReferenceY - LBY > 0 && abs(ReferenceX - MCX) < abs(ReferenceY - MCY)) //Area D

				{
					ReferenceXSub = ReferenceY - 4 * N;
					ReferenceYSub = ReferenceX - 3 * N;
					//---------------------------Projection Transform----------------------------------//
					ReferenceXSubPro = (N * ReferenceXSub) / (N + ReferenceXSub);
					ReferenceYSubPro = (N * ReferenceYSub) / (N + ReferenceXSub);
					//---------------------------Coordinates of Reference Block after modification in picture------------------//
					ReferenceXProjection = 2 * N - ReferenceXSubPro;
					ReferenceYProjection = 5 * N + ReferenceYSubPro;
					RotationIdx = 1;
					//-------------------------calculate half-block size-----------------------//
					height1 = height;
					height2 = 0;
					width2 = 0;
					width1 = width;
				}
				else
				{
					ReferenceXProjection = ReferenceX;
					ReferenceYProjection = ReferenceY;
					RotationIdx = 0;
					height1 = height;
					height2 = 0;
					width2 = 0;
					width1 = width;
				}
				break;

			case 4:// current face belong to face 4
				//-------------------------------Get coordinates of four corners of face 4--------------------------------//
				LAX = 4 * N; LAY = 2 * N;
				RAX = 6 * N - 1; RAY = 2 * N;
				LBX = 4 * N; LBY = 4 * N - 1;
				RBX = 6 * N - 1; RBY = 4 * N - 1;
				MCX = (LAX + RAX + 1) / 2;
				MCY = (LAY + 1 + LBY) / 2;
				if (ReferenceX - RAX > 0 && abs(ReferenceX - MCX) > abs(ReferenceY - MCY)) //Area A
				{
					ReferenceXSub = ReferenceX - 6 * N;
					ReferenceYSub = 3 * N + 1 - ReferenceY;
					//---------------------------Projection Transform----------------------------------//
					ReferenceXSubPro = (N * ReferenceXSub) / (N + ReferenceXSub);
					ReferenceYSubPro = (N * ReferenceYSub) / (N + ReferenceXSub);
					//---------------------------Coordinates of Reference Block after modification in picture------------------//
					ReferenceXProjection = 6 * N + ReferenceXSubPro;
					ReferenceYProjection = 3 * N + ReferenceYSubPro - 1;
					RotationIdx = 0;
					//-------------------------calculate half-block size-----------------------//
					height1 = height;
					height2 = 0;
					width2 = 0;
					width1 = width;
				}
				else if (ReferenceY - LAY < 0 && abs(ReferenceX - MCX) < abs(ReferenceY - MCY))  //Area B
				{
					ReferenceXSub = 2 * N + 1 - ReferenceY;
					ReferenceYSub = 5 * N + 1 - ReferenceX;
					//---------------------------Projection Transform----------------------------------//
					ReferenceXSubPro = (N * ReferenceXSub) / (N + ReferenceXSub);
					ReferenceYSubPro = (N * ReferenceYSub) / (N + ReferenceXSub);
					//---------------------------Coordinates of Reference Block after modification in picture------------------//
					ReferenceXProjection = N + ReferenceYSubPro - 1;
					ReferenceYProjection = ReferenceXSubPro - 1;
					RotationIdx = 2;
					//-------------------------calculate half-block size-----------------------//
					width2 = width;
					width1 = width;
					if (ReferenceY>0)
					{
						height2 = (3840 - ReferenceY % 3840 + 8) / 16 * 4;
					}
					else
					{
						height2 = (abs(ReferenceY) % 3840 + 8) / 16 * 4;
					}
					height1 = height - height2;
				}
				else if (ReferenceX - LAX < 0 && abs(ReferenceX - MCX) > abs(ReferenceY - MCY))  //Area C
				{
					ReferenceXSub = 4 * N + 1 - ReferenceX;
					ReferenceYSub = ReferenceY - 3 * N;
					//---------------------------Projection Transform----------------------------------//
					ReferenceXSubPro = (N * ReferenceXSub) / (N + ReferenceXSub);
					ReferenceYSubPro = (N * ReferenceYSub) / (N + ReferenceXSub);
					//---------------------------Coordinates of Reference Block after modification in picture------------------//
					ReferenceXProjection = 4 * N - ReferenceXSubPro - 1;
					ReferenceYProjection = 3 * N + ReferenceYSubPro;
					RotationIdx = 0;
					//-------------------------calculate half-block size-----------------------//
					height1 = height;
					height2 = height;
					if (ReferenceX>0)
					{
						width2 = (3840 - ReferenceX % 3840 + 8) / 16 * 4;
					}
					else
					{
						width2 = (abs(ReferenceX) % 3840 + 8) / 16 * 4;
					}
					width1 = width - width2;
				}
				else if (ReferenceY - LBY > 0 && abs(ReferenceX - MCX) < abs(ReferenceY - MCY)) //Area D
				{
					ReferenceXSub = ReferenceY - 4 * N;
					ReferenceYSub = ReferenceX - 5 * N;
					//---------------------------Projection Transform----------------------------------//
					ReferenceXSubPro = (N * ReferenceXSub) / (N + ReferenceXSub);
					ReferenceYSubPro = (N * ReferenceYSub) / (N + ReferenceXSub);
					//--------------------------Coordinates of Reference Block after modification in picture------------------//
					ReferenceXProjection = N - ReferenceYSubPro;
					ReferenceYProjection = 6 * N - ReferenceXSubPro;
					RotationIdx = 2;
					//-------------------------calculate half-block size-----------------------//
					height1 = height;
					height2 = 0;
					width2 = 0;
					width1 = width;
				}
				else
				{
					ReferenceXProjection = ReferenceX;
					ReferenceYProjection = ReferenceY;
					RotationIdx = 0;
					height1 = height;
					height2 = 0;
					width2 = 0;
					width1 = width;
				}
				break;

			case 5:// current face belong to face 5
				//-------------------------------Get coordinates of four corners of face 5--------------------------------//
				LAX = 6 * N; LAY = 2 * N;
				RAX = 8 * N - 1; RAY = 2 * N;
				LBX = 6 * N; LBY = 4 * N - 1;
				RBX = 8 * N - 1; RBY = 4 * N - 1;
				MCX = (LAX + RAX + 1) / 2;
				MCY = (LAY + 1 + LBY) / 2;
				if (ReferenceX - RAX > 0 && abs(ReferenceX - MCX) > abs(ReferenceY - MCY)) //Area A
				{
					ReferenceXSub = ReferenceX - 8 * N;
					ReferenceYSub = 3 * N + 1 - ReferenceY;
					//---------------------------Projection Transform----------------------------------//
					ReferenceXSubPro = (N * ReferenceXSub) / (N + ReferenceXSub);
					ReferenceYSubPro = (N * ReferenceYSub) / (N + ReferenceXSub);
					//---------------------------Coordinates of Reference Block after modification in picture------------------//
					ReferenceXProjection = ReferenceXSubPro;
					ReferenceYProjection = 3 * N + ReferenceYSubPro - 1;
					RotationIdx = 0;
					//-------------------------calculate half-block size-----------------------//
					height1 = height;
					height2 = 0;
					width2 = 0;
					width1 = width;
				}
				else if (ReferenceY - LAY < 0 && abs(ReferenceX - MCX) < abs(ReferenceY - MCY))  //Area B
				{
					ReferenceXSub = 2 * N + 1 - ReferenceY;
					ReferenceYSub = 7 * N + 1 - ReferenceX;
					//---------------------------Projection Transform----------------------------------//
					ReferenceXSubPro = (N * ReferenceXSub) / (N + ReferenceXSub);
					ReferenceYSubPro = (N * ReferenceYSub) / (N + ReferenceXSub);
					//---------------------------Coordinates of Reference Block after modification in picture------------------//
					ReferenceXProjection = ReferenceXSubPro - 1;
					ReferenceYProjection = N - ReferenceYSubPro - 1;
					RotationIdx = 1;
					//-------------------------calculate half-block size-----------------------//
					width2 = width;
					width1 = width;
					if (ReferenceY>0)
					{
						height2 = (3840 - ReferenceY % 3840 + 8) / 16 * 4;
					}
					else
					{
						height2 = (abs(ReferenceY) % 3840 + 8) / 16 * 4;
					}
					height1 = height - height2;
				}
				else if (ReferenceX - LAX < 0 && abs(ReferenceX - MCX) > abs(ReferenceY - MCY))  //Area C
				{
					ReferenceXSub = 6 * N + 1 - ReferenceX;
					ReferenceYSub = ReferenceY - 3 * N;
					//---------------------------Projection Transform----------------------------------//
					ReferenceXSubPro = (N * ReferenceXSub) / (N + ReferenceXSub);
					ReferenceYSubPro = (N * ReferenceYSub) / (N + ReferenceXSub);
					//---------------------------Coordinates of Reference Block after modification in picture------------------//
					ReferenceXProjection = 6 * N - ReferenceXSubPro - 1;
					ReferenceYProjection = 3 * N + ReferenceYSubPro;
					RotationIdx = 0;
					//-------------------------calculate half-block size-----------------------//
					height1 = height;
					height2 = height;
					if (ReferenceX>0)
					{
						width2 = (3840 - ReferenceX % 3840 + 8) / 16 * 4;
					}
					else
					{
						width2 = (abs(ReferenceX) % 3840 + 8) / 16 * 4;
					}
					width1 = width - width2;
				}
				else if (ReferenceY - LBY > 0 && abs(ReferenceX - MCX) < abs(ReferenceY - MCY)) //Area D
				{
					ReferenceXSub = ReferenceY - 4 * N;
					ReferenceYSub = ReferenceX - 7 * N;
					//---------------------------Projection Transform----------------------------------//
					ReferenceXSubPro = (N * ReferenceXSub) / (N + ReferenceXSub);
					ReferenceYSubPro = (N * ReferenceYSub) / (N + ReferenceXSub);
					//---------------------------Coordinates of Reference Block after modification in picture------------------//
					ReferenceXProjection = ReferenceXSubPro;
					ReferenceYProjection = 5 * N - ReferenceYSubPro;
					RotationIdx = 3;
					//-------------------------calculate half-block size-----------------------//
					height1 = height;
					height2 = 0;
					width2 = 0;
					width1 = width;
				}
				else
				{
					ReferenceXProjection = ReferenceX;
					ReferenceYProjection = ReferenceY;
					RotationIdx = 0;
					height1 = height;
					height2 = 0;
					width2 = 0;
					width1 = width;
				}
				break;

			case 6:// current face belong to face 6
				//-------------------------------Get coordinates of four corners of face 6--------------------------------//
				LAX = 0; LAY = 4 * N;
				RAX = 2 * N - 1; RAY = 4 * N;
				LBX = 0; LBY = 6 * N - 1;
				RBX = 2 * N - 1; RBY = 6 * N - 1;
				MCX = (LAX + RAX + 1) / 2;
				MCY = (LAY + 1 + LBY) / 2;
				if (ReferenceX - RAX > 0 && abs(ReferenceX - MCX) > abs(ReferenceY - MCY)) //Area A
				{
					ReferenceXSub = ReferenceX - 2 * N;
					ReferenceYSub = 5 * N + 1 - ReferenceY;
					//---------------------------Projection Transform----------------------------------//
					ReferenceXSubPro = (N * ReferenceXSub) / (N + ReferenceXSub);
					ReferenceYSubPro = (N * ReferenceYSub) / (N + ReferenceXSub);
					//---------------------------Coordinates of Reference Block after modification in picture------------------//
					ReferenceXProjection = 3 * N - ReferenceYSubPro;
					ReferenceYProjection = 4 * N - ReferenceXSubPro - 1;
					RotationIdx = 3;
					//-------------------------calculate half-block size-----------------------//
					height1 = height;
					height2 = 0;
					width2 = 0;
					width1 = width;
				}
				else if (ReferenceY - LAY < 0 && abs(ReferenceX - MCX) < abs(ReferenceY - MCY))  //Area B
				{
					ReferenceXSub = 4 * N + 1 - ReferenceY;
					ReferenceYSub = N + 1 - ReferenceX;
					//---------------------------Projection Transform----------------------------------//
					ReferenceXSubPro = (N * ReferenceXSub) / (N + ReferenceXSub);
					ReferenceYSubPro = (N * ReferenceYSub) / (N + ReferenceXSub);
					//---------------------------Coordinates of Reference Block after modification in picture------------------//
					ReferenceXProjection = N - ReferenceYSubPro - 1;
					ReferenceYProjection = 4 * N - ReferenceXSubPro - 1;
					RotationIdx = 0;
					//-------------------------calculate half-block size-----------------------//
					width2 = width;
					width1 = width;
					if (ReferenceY>0)
					{
						height2 = (3840 - ReferenceY % 3840 + 8) / 16 * 4;
					}
					else
					{
						height2 = (abs(ReferenceY) % 3840 + 8) / 16 * 4;
					}
					height1 = height - height2;
				}
				else if (ReferenceX - LAX < 0 && abs(ReferenceX - MCX) > abs(ReferenceY - MCY))  //Area C
				{
					ReferenceXSub = 1 - ReferenceX;
					ReferenceYSub = ReferenceY - 5 * N;
					//---------------------------Projection Transform----------------------------------//
					ReferenceXSubPro = (N * ReferenceXSub) / (N + ReferenceXSub);
					ReferenceYSubPro = (N * ReferenceYSub) / (N + ReferenceXSub);
					//---------------------------Coordinates of Reference Block after modification in picture------------------//
					ReferenceXProjection = 7 * N - ReferenceYSubPro - 1;
					ReferenceYProjection = 4 * N - ReferenceXSubPro;
					RotationIdx = 1;
					//-------------------------calculate half-block size-----------------------//
					height1 = height;
					height2 = height;
					if (ReferenceX>0)
					{
						width2 = (3840 - ReferenceX % 3840 + 8) / 16 * 4;
					}
					else
					{
						width2 = (abs(ReferenceX) % 3840 + 8) / 16 * 4;
					}
					width1 = width - width2;
				}
				else if (ReferenceY - LBY > 0 && abs(ReferenceX - MCX) < abs(ReferenceY - MCY)) //Area D
				{
					ReferenceXSub = ReferenceY - 6 * N;
					ReferenceYSub = ReferenceX - N;
					//---------------------------Projection Transform----------------------------------//
					ReferenceXSubPro = (N * ReferenceXSub) / (N + ReferenceXSub);
					ReferenceYSubPro = (N * ReferenceYSub) / (N + ReferenceXSub);
					//---------------------------Coordinates of Reference Block after modification in picture------------------//
					ReferenceXProjection = 5 * N - ReferenceYSubPro;
					ReferenceYProjection = 4 * N - ReferenceXSubPro;
					RotationIdx = 2;
					//-------------------------calculate half-block size-----------------------//
					height1 = height;
					height2 = 0;
					width2 = 0;
					width1 = width;
				}
				else
				{
					ReferenceXProjection = ReferenceX;
					ReferenceYProjection = ReferenceY;
					RotationIdx = 0;
					height1 = height;
					height2 = 0;
					width2 = 0;
					width1 = width;
				}
				break;
			case 0:
				ReferenceXProjection = ReferenceX;
				ReferenceYProjection = ReferenceY;
				RotationIdx = 0;
				height1 = height;
				height2 = 0;
				width2 = 0;
				width1 = width;
				break;
			case 8:

				cerr << "FaceNum not assigned.2/n" << endl;

				break;
			default:

				cerr << "FaceNum = " << FaceNum << "/n" << endl;

				break;
			}
			Int mvX_2 = ReferenceXProjection - CurrentPUX;
			Int mvY_2 = ReferenceYProjection - CurrentPUY;
			TComMv mv1(0, 0);
			mv1.setHor(mvX_2);
			mv1.setVer(mvY_2);

			assert(height1*width1 + width2*height2 == width*height);
			assert(height1 >= 0 && width1 >= 0 && height2 >= 0 && width2 >= 0);

			xPredInterBlk_PBSMC(compID, cu, refPic, partAddr, mv1, width2, height2, dstPic, bi, bitDepth, RotationIdx);
			xPredInterBlk(compID, cu, refPic, g_auiRasterToZscan[g_auiZscanToRaster[partAddr] + (width2%width) / 4 + (height2%height) * 4], mv, width1, height1, dstPic, bi, bitDepth);

		}
	}

	else
	{
		for (Int i = 0; i < SBHeight; i++)
		{
			for (Int j = 0; j < SBWidth; j++)
			{
				CurrentPUX = TempCurrentPUX + j * 4 * 4;
				CurrentPUY = TempCurrentPUY + i * 4 * 4;
				//------------calculate the current PU belongs to which face in picture with the format of CMP4x3----------------------------// 
				ReferenceX = CurrentPUX + mvX;
				ReferenceY = CurrentPUY + mvY;
				switch (FaceNum)
				{
				case 1:// current face belong to face 1
					//-------------------------------Get coordinates of four corners of face 1--------------------------------//
					LAX = 0; LAY = 0;
					RAX = 2 * N - 1; RAY = 0;
					LBX = 0; LBY = 2 * N - 1;
					RBX = 2 * N - 1; RBY = 2 * N - 1;
					MCX = (LAX + RAX + 1) / 2;
					MCY = (LAY + 1 + LBY) / 2;
					if (ReferenceX - RAX > 0 && abs(ReferenceX - MCX) > abs(ReferenceY - MCY)) //Area A
					{
						ReferenceXSub = ReferenceX - 2 * N;
						ReferenceYSub = N + 1 - ReferenceY;
						//---------------------------Projection Transform----------------------------------//
						ReferenceXSubPro = (N * ReferenceXSub) / (N + ReferenceXSub);
						ReferenceYSubPro = (N * ReferenceYSub) / (N + ReferenceXSub);
						//---------------------------Coordinates of Reference Block after modification in picture------------------//
						ReferenceXProjection = 3 * N + ReferenceYSubPro;
						ReferenceYProjection = 2 * N + ReferenceXSubPro - 1;
						RotationIdx = 1;
					}
					else if (ReferenceY - LAY < 0 && abs(ReferenceX - MCX) < abs(ReferenceY - MCY))  //Area B
					{
						ReferenceXSub = 1 - ReferenceY;
						ReferenceYSub = N + 1 - ReferenceX;
						//---------------------------Projection Transform----------------------------------//
						ReferenceXSubPro = (N * ReferenceXSub) / (N + ReferenceXSub);
						ReferenceYSubPro = (N * ReferenceYSub) / (N + ReferenceXSub);
						//---------------------------Coordinates of Reference Block after modification in picture------------------//
						ReferenceXProjection = 5 * N + ReferenceXSubPro - 1;
						ReferenceYProjection = 2 * N + ReferenceYSubPro - 1;
						RotationIdx = 2;
					}
					else if (ReferenceX - LAX < 0 && abs(ReferenceX - MCX) > abs(ReferenceY - MCY))  //Area C
					{
						ReferenceXSub = 1 - ReferenceX;
						ReferenceYSub = ReferenceY - N;
						//---------------------------Projection Transform----------------------------------//
						ReferenceXSubPro = (N * ReferenceXSub) / (N + ReferenceXSub);
						ReferenceYSubPro = (N * ReferenceYSub) / (N + ReferenceXSub);
						//---------------------------Coordinates of Reference Block after modification in picture------------------//
						ReferenceXProjection = 7 * N + ReferenceYSubPro - 1;
						ReferenceYProjection = 2 * N + ReferenceXSubPro;
						RotationIdx = 3;
					}
					else if (ReferenceY - LBY > 0 && abs(ReferenceX - MCX) < abs(ReferenceY - MCY)) //Area D
					{
						ReferenceXSub = ReferenceY - 2 * N;
						ReferenceYSub = ReferenceX - N;
						//---------------------------Projection Transform----------------------------------//
						ReferenceXSubPro = (N * ReferenceXSub) / (N + ReferenceXSub);
						ReferenceYSubPro = (N * ReferenceYSub) / (N + ReferenceXSub);
						//---------------------------Coordinates of Reference Block after modification in picture------------------//
						ReferenceXProjection = N + ReferenceYSubPro;
						ReferenceYProjection = 2 * N + ReferenceXSubPro;
						RotationIdx = 0;
					}
					else
					{
						ReferenceXProjection = ReferenceX;
						ReferenceYProjection = ReferenceY;
						RotationIdx = 0;
					}
					break;

				case 2:// current face belong to face 2
					//-------------------------------Get coordinates of four corners of face 2--------------------------------//
					LAX = 0; LAY = 2 * N;
					RAX = 2 * N - 1; RAY = 2 * N;
					LBX = 0; LBY = 4 * N - 1;
					RBX = 2 * N - 1; RBY = 4 * N - 1;
					MCX = (LAX + RAX + 1) / 2;
					MCY = (LAY + 1 + LBY) / 2;
					if (ReferenceX - RAX > 0 && abs(ReferenceX - MCX) > abs(ReferenceY - MCY)) //Area A
					{
						ReferenceXSub = ReferenceX - 2 * N;
						ReferenceYSub = 3 * N + 1 - ReferenceY;
						//---------------------------Projection Transform----------------------------------//
						ReferenceXSubPro = (N * ReferenceXSub) / (N + ReferenceXSub);
						ReferenceYSubPro = (N * ReferenceYSub) / (N + ReferenceXSub);
						//---------------------------Coordinates of Reference Block after modification in picture------------------//
						ReferenceXProjection = 2 * N + ReferenceXSubPro;
						ReferenceYProjection = 3 * N - ReferenceYSubPro - 1;
						RotationIdx = 0;
					}
					else if (ReferenceY - LAY < 0 && abs(ReferenceX - MCX) < abs(ReferenceY - MCY))  //Area B
					{
						ReferenceXSub = 2 * N + 1 - ReferenceY;
						ReferenceYSub = N + 1 - ReferenceX;
						//---------------------------Projection Transform----------------------------------//
						ReferenceXSubPro = (N * ReferenceXSub) / (N + ReferenceXSub);
						ReferenceYSubPro = (N * ReferenceYSub) / (N + ReferenceXSub);
						//---------------------------Coordinates of Reference Block after modification in picture------------------//
						ReferenceXProjection = N - ReferenceYSubPro - 1;
						ReferenceYProjection = 2 * N - ReferenceXSubPro - 1;
						RotationIdx = 0;
					}
					else if (ReferenceX - LAX < 0 && abs(ReferenceX - MCX) > abs(ReferenceY - MCY))  //Area C
					{
						ReferenceXSub = 1 - ReferenceX;
						ReferenceYSub = ReferenceY - 3 * N;
						//---------------------------Projection Transform----------------------------------//
						ReferenceXSubPro = (N * ReferenceXSub) / (N + ReferenceXSub);
						ReferenceYSubPro = (N * ReferenceYSub) / (N + ReferenceXSub);
						//---------------------------Coordinates of Reference Block after modification in picture------------------//
						ReferenceXProjection = 8 * N - ReferenceXSubPro - 1;
						ReferenceYProjection = 3 * N + ReferenceYSubPro;
						RotationIdx = 0;
					}
					else if (ReferenceY - LBY > 0 && abs(ReferenceX - MCX) < abs(ReferenceY - MCY)) //Area D
					{
						ReferenceXSub = ReferenceY - 4 * N;
						ReferenceYSub = ReferenceX - N;
						//---------------------------Projection Transform----------------------------------//
						ReferenceXSubPro = (N * ReferenceXSub) / (N + ReferenceXSub);
						ReferenceYSubPro = (N * ReferenceYSub) / (N + ReferenceXSub);
						//---------------------------Coordinates of Reference Block after modification in picture------------------//
						ReferenceXProjection = N + ReferenceYSubPro;
						ReferenceYProjection = 2 * N + ReferenceXSubPro;
						RotationIdx = 0;
					}
					else
					{
						ReferenceXProjection = ReferenceX;
						ReferenceYProjection = ReferenceY;
						RotationIdx = 0;
					}
					break;

				case 3:// current face belong to face 3
					//-------------------------------Get coordinates of four corners of face 3--------------------------------//
					LAX = 2 * N; LAY = 2 * N;
					RAX = 4 * N - 1; RAY = 2 * N;
					LBX = 2 * N; LBY = 4 * N - 1;
					RBX = 4 * N - 1; RBY = 4 * N - 1;
					MCX = (LAX + RAX + 1) / 2;
					MCY = (LAY + 1 + LBY) / 2;
					if (ReferenceX - RAX > 0 && abs(ReferenceX - MCX) > abs(ReferenceY - MCY)) //Area A
					{
						ReferenceXSub = ReferenceX - 4 * N;
						ReferenceYSub = 3 * N + 1 - ReferenceY;
						//---------------------------Projection Transform----------------------------------//
						ReferenceXSubPro = (N * ReferenceXSub) / (N + ReferenceXSub);
						ReferenceYSubPro = (N * ReferenceYSub) / (N + ReferenceXSub);
						//---------------------------Coordinates of Reference Block after modification in picture------------------//
						ReferenceXProjection = 4 * N + ReferenceXSubPro;
						ReferenceYProjection = 3 * N + ReferenceYSubPro - 1;
						RotationIdx = 0;
					}
					else if (ReferenceY - LAY < 0 && abs(ReferenceX - MCX) < abs(ReferenceY - MCY))  //Area B
					{
						ReferenceXSub = 2 * N + 1 - ReferenceY;
						ReferenceYSub = 3 * N + 1 - ReferenceX;
						//---------------------------Projection Transform----------------------------------//
						ReferenceXSubPro = (N * ReferenceXSub) / (N + ReferenceXSub);
						ReferenceYSubPro = (N * ReferenceYSub) / (N + ReferenceXSub);
						//---------------------------Coordinates of Reference Block after modification in picture------------------//
						ReferenceXProjection = 2 * N - ReferenceXSubPro - 1;
						ReferenceYProjection = N + ReferenceYSubPro - 1;
						RotationIdx = 3;
					}
					else if (ReferenceX - LAX < 0 && abs(ReferenceX - MCX) > abs(ReferenceY - MCY))  //Area C
					{
						ReferenceXSub = 2 * N + 1 - ReferenceX;
						ReferenceYSub = ReferenceY - 3 * N;
						//---------------------------Projection Transform----------------------------------//
						ReferenceXSubPro = (N * ReferenceXSub) / (N + ReferenceXSub);
						ReferenceYSubPro = (N * ReferenceYSub) / (N + ReferenceXSub);
						//---------------------------Coordinates of Reference Block after modification in picture------------------//
						ReferenceXProjection = 2 * N - ReferenceXSubPro - 1;
						ReferenceYProjection = 3 * N + ReferenceYSubPro;
						RotationIdx = 0;
					}
					else if (ReferenceY - LBY > 0 && abs(ReferenceX - MCX) < abs(ReferenceY - MCY)) //Area D

					{
						ReferenceXSub = ReferenceY - 4 * N;
						ReferenceYSub = ReferenceX - 3 * N;
						//---------------------------Projection Transform----------------------------------//
						ReferenceXSubPro = (N * ReferenceXSub) / (N + ReferenceXSub);
						ReferenceYSubPro = (N * ReferenceYSub) / (N + ReferenceXSub);
						//---------------------------Coordinates of Reference Block after modification in picture------------------//
						ReferenceXProjection = 2 * N - ReferenceXSubPro;
						ReferenceYProjection = 5 * N + ReferenceYSubPro;
						RotationIdx = 1;
					}
					else
					{
						ReferenceXProjection = ReferenceX;
						ReferenceYProjection = ReferenceY;
						RotationIdx = 0;
					}
					break;

				case 4:// current face belong to face 4
					//-------------------------------Get coordinates of four corners of face 4--------------------------------//
					LAX = 4 * N; LAY = 2 * N;
					RAX = 6 * N - 1; RAY = 2 * N;
					LBX = 4 * N; LBY = 4 * N - 1;
					RBX = 6 * N - 1; RBY = 4 * N - 1;
					MCX = (LAX + RAX + 1) / 2;
					MCY = (LAY + 1 + LBY) / 2;
					if (ReferenceX - RAX > 0 && abs(ReferenceX - MCX) > abs(ReferenceY - MCY)) //Area A
					{
						ReferenceXSub = ReferenceX - 6 * N;
						ReferenceYSub = 3 * N + 1 - ReferenceY;
						//---------------------------Projection Transform----------------------------------//
						ReferenceXSubPro = (N * ReferenceXSub) / (N + ReferenceXSub);
						ReferenceYSubPro = (N * ReferenceYSub) / (N + ReferenceXSub);
						//---------------------------Coordinates of Reference Block after modification in picture------------------//
						ReferenceXProjection = 6 * N + ReferenceXSubPro;
						ReferenceYProjection = 3 * N + ReferenceYSubPro - 1;
						RotationIdx = 0;
					}
					else if (ReferenceY - LAY < 0 && abs(ReferenceX - MCX) < abs(ReferenceY - MCY))  //Area B
					{
						ReferenceXSub = 2 * N + 1 - ReferenceY;
						ReferenceYSub = 5 * N + 1 - ReferenceX;
						//---------------------------Projection Transform----------------------------------//
						ReferenceXSubPro = (N * ReferenceXSub) / (N + ReferenceXSub);
						ReferenceYSubPro = (N * ReferenceYSub) / (N + ReferenceXSub);
						//---------------------------Coordinates of Reference Block after modification in picture------------------//
						ReferenceXProjection = N + ReferenceYSubPro - 1;
						ReferenceYProjection = ReferenceXSubPro - 1;
						RotationIdx = 2;
					}
					else if (ReferenceX - LAX < 0 && abs(ReferenceX - MCX) > abs(ReferenceY - MCY))  //Area C
					{
						ReferenceXSub = 4 * N + 1 - ReferenceX;
						ReferenceYSub = ReferenceY - 3 * N;
						//---------------------------Projection Transform----------------------------------//
						ReferenceXSubPro = (N * ReferenceXSub) / (N + ReferenceXSub);
						ReferenceYSubPro = (N * ReferenceYSub) / (N + ReferenceXSub);
						//---------------------------Coordinates of Reference Block after modification in picture------------------//
						ReferenceXProjection = 4 * N - ReferenceXSubPro - 1;
						ReferenceYProjection = 3 * N + ReferenceYSubPro;
						RotationIdx = 0;
					}
					else if (ReferenceY - LBY > 0 && abs(ReferenceX - MCX) < abs(ReferenceY - MCY)) //Area D
					{
						ReferenceXSub = ReferenceY - 4 * N;
						ReferenceYSub = ReferenceX - 5 * N;
						//---------------------------Projection Transform----------------------------------//
						ReferenceXSubPro = (N * ReferenceXSub) / (N + ReferenceXSub);
						ReferenceYSubPro = (N * ReferenceYSub) / (N + ReferenceXSub);
						//--------------------------Coordinates of Reference Block after modification in picture------------------//
						ReferenceXProjection = N - ReferenceYSubPro;
						ReferenceYProjection = 6 * N - ReferenceXSubPro;
						RotationIdx = 2;
					}
					else
					{
						ReferenceXProjection = ReferenceX;
						ReferenceYProjection = ReferenceY;
						RotationIdx = 0;
					}
					break;

				case 5:// current face belong to face 5
					//-------------------------------Get coordinates of four corners of face 5--------------------------------//
					LAX = 6 * N; LAY = 2 * N;
					RAX = 8 * N - 1; RAY = 2 * N;
					LBX = 6 * N; LBY = 4 * N - 1;
					RBX = 8 * N - 1; RBY = 4 * N - 1;
					MCX = (LAX + RAX + 1) / 2;
					MCY = (LAY + 1 + LBY) / 2;
					if (ReferenceX - RAX > 0 && abs(ReferenceX - MCX) > abs(ReferenceY - MCY)) //Area A
					{
						ReferenceXSub = ReferenceX - 8 * N;
						ReferenceYSub = 3 * N + 1 - ReferenceY;
						//---------------------------Projection Transform----------------------------------//
						ReferenceXSubPro = (N * ReferenceXSub) / (N + ReferenceXSub);
						ReferenceYSubPro = (N * ReferenceYSub) / (N + ReferenceXSub);
						//---------------------------Coordinates of Reference Block after modification in picture------------------//
						ReferenceXProjection = ReferenceXSubPro;
						ReferenceYProjection = 3 * N + ReferenceYSubPro - 1;
						RotationIdx = 0;
					}
					else if (ReferenceY - LAY < 0 && abs(ReferenceX - MCX) < abs(ReferenceY - MCY))  //Area B
					{
						ReferenceXSub = 2 * N + 1 - ReferenceY;
						ReferenceYSub = 7 * N + 1 - ReferenceX;
						//---------------------------Projection Transform----------------------------------//
						ReferenceXSubPro = (N * ReferenceXSub) / (N + ReferenceXSub);
						ReferenceYSubPro = (N * ReferenceYSub) / (N + ReferenceXSub);
						//---------------------------Coordinates of Reference Block after modification in picture------------------//
						ReferenceXProjection = ReferenceXSubPro - 1;
						ReferenceYProjection = N - ReferenceYSubPro - 1;
						RotationIdx = 1;
					}
					else if (ReferenceX - LAX < 0 && abs(ReferenceX - MCX) > abs(ReferenceY - MCY))  //Area C
					{
						ReferenceXSub = 6 * N + 1 - ReferenceX;
						ReferenceYSub = ReferenceY - 3 * N;
						//---------------------------Projection Transform----------------------------------//
						ReferenceXSubPro = (N * ReferenceXSub) / (N + ReferenceXSub);
						ReferenceYSubPro = (N * ReferenceYSub) / (N + ReferenceXSub);
						//---------------------------Coordinates of Reference Block after modification in picture------------------//
						ReferenceXProjection = 6 * N - ReferenceXSubPro - 1;
						ReferenceYProjection = 3 * N + ReferenceYSubPro;
						RotationIdx = 0;
					}
					else if (ReferenceY - LBY > 0 && abs(ReferenceX - MCX) < abs(ReferenceY - MCY)) //Area D
					{
						ReferenceXSub = ReferenceY - 4 * N;
						ReferenceYSub = ReferenceX - 7 * N;
						//---------------------------Projection Transform----------------------------------//
						ReferenceXSubPro = (N * ReferenceXSub) / (N + ReferenceXSub);
						ReferenceYSubPro = (N * ReferenceYSub) / (N + ReferenceXSub);
						//---------------------------Coordinates of Reference Block after modification in picture------------------//
						ReferenceXProjection = ReferenceXSubPro;
						ReferenceYProjection = 5 * N - ReferenceYSubPro;
						RotationIdx = 3;
					}
					else
					{
						ReferenceXProjection = ReferenceX;
						ReferenceYProjection = ReferenceY;
						RotationIdx = 0;
					}
					break;

				case 6:// current face belong to face 6
					//-------------------------------Get coordinates of four corners of face 6--------------------------------//
					LAX = 0; LAY = 4 * N;
					RAX = 2 * N - 1; RAY = 4 * N;
					LBX = 0; LBY = 6 * N - 1;
					RBX = 2 * N - 1; RBY = 6 * N - 1;
					MCX = (LAX + RAX + 1) / 2;
					MCY = (LAY + 1 + LBY) / 2;
					if (ReferenceX - RAX > 0 && abs(ReferenceX - MCX) > abs(ReferenceY - MCY)) //Area A
					{
						ReferenceXSub = ReferenceX - 2 * N;
						ReferenceYSub = 5 * N + 1 - ReferenceY;
						//---------------------------Projection Transform----------------------------------//
						ReferenceXSubPro = (N * ReferenceXSub) / (N + ReferenceXSub);
						ReferenceYSubPro = (N * ReferenceYSub) / (N + ReferenceXSub);
						//---------------------------Coordinates of Reference Block after modification in picture------------------//
						ReferenceXProjection = 3 * N - ReferenceYSubPro;
						ReferenceYProjection = 4 * N - ReferenceXSubPro - 1;
						RotationIdx = 3;
					}
					else if (ReferenceY - LAY < 0 && abs(ReferenceX - MCX) < abs(ReferenceY - MCY))  //Area B
					{
						ReferenceXSub = 4 * N + 1 - ReferenceY;
						ReferenceYSub = N + 1 - ReferenceX;
						//---------------------------Projection Transform----------------------------------//
						ReferenceXSubPro = (N * ReferenceXSub) / (N + ReferenceXSub);
						ReferenceYSubPro = (N * ReferenceYSub) / (N + ReferenceXSub);
						//---------------------------Coordinates of Reference Block after modification in picture------------------//
						ReferenceXProjection = N - ReferenceYSubPro - 1;
						ReferenceYProjection = 4 * N - ReferenceXSubPro - 1;
						RotationIdx = 0;
					}
					else if (ReferenceX - LAX < 0 && abs(ReferenceX - MCX) > abs(ReferenceY - MCY))  //Area C
					{
						ReferenceXSub = 1 - ReferenceX;
						ReferenceYSub = ReferenceY - 5 * N;
						//---------------------------Projection Transform----------------------------------//
						ReferenceXSubPro = (N * ReferenceXSub) / (N + ReferenceXSub);
						ReferenceYSubPro = (N * ReferenceYSub) / (N + ReferenceXSub);
						//---------------------------Coordinates of Reference Block after modification in picture------------------//
						ReferenceXProjection = 7 * N - ReferenceYSubPro - 1;
						ReferenceYProjection = 4 * N - ReferenceXSubPro;
						RotationIdx = 1;
					}
					else if (ReferenceY - LBY > 0 && abs(ReferenceX - MCX) < abs(ReferenceY - MCY)) //Area D
					{
						ReferenceXSub = ReferenceY - 6 * N;
						ReferenceYSub = ReferenceX - N;
						//---------------------------Projection Transform----------------------------------//
						ReferenceXSubPro = (N * ReferenceXSub) / (N + ReferenceXSub);
						ReferenceYSubPro = (N * ReferenceYSub) / (N + ReferenceXSub);
						//---------------------------Coordinates of Reference Block after modification in picture------------------//
						ReferenceXProjection = 5 * N - ReferenceYSubPro;
						ReferenceYProjection = 4 * N - ReferenceXSubPro;
						RotationIdx = 2;
					}
					else
					{
						ReferenceXProjection = ReferenceX;
						ReferenceYProjection = ReferenceY;
						RotationIdx = 0;
					}
					break;
				case 0:
					ReferenceXProjection = ReferenceX;
					ReferenceYProjection = ReferenceY;
					RotationIdx = 0;
					break;
				case 8:

					cerr << "FaceNum not assigned.2/n" << endl;

					break;
				default:

					cerr << "FaceNum = " << FaceNum << "/n" << endl;

					break;
				}
				Int mvX_2 = ReferenceXProjection - CurrentPUX;
				Int mvY_2 = ReferenceYProjection - CurrentPUY;
				TComMv mv1(0, 0);
				mv1.setHor(mvX_2);
				mv1.setVer(mvY_2);
				xPredInterBlk_PBSMC(compID, cu, refPic, g_auiRasterToZscan[g_auiZscanToRaster[partAddr] + j + i * 16], mv1, 4, 4, dstPic, bi, bitDepth, RotationIdx);

			}// end of for (Int j = 0; j < SBWidth; j++)       
		}//end of for (Int i = 0; i< SBHeight; i++) 
	}// end of else

}
#else

Void TComPrediction::xPredInterBlk_Top(const ComponentID compID, TComDataCU *cu, TComPicYuv *refPic, UInt partAddr, TComMv *mv, Int width, Int height, TComYuv *dstPic, Bool bi, const Int bitDepth)
{
	Int shiftHor = (2 + refPic->getComponentScaleX(compID));
	Int shiftVer = (2 + refPic->getComponentScaleY(compID));
	Int RotationIdx = 0;
	Int CtuRsAddr = cu->getCtuRsAddr();  //grab CTU raster scan order
	Int CtuNumInRow = refPic->getWidth(COMPONENT_Y) / 64;
	//-----------------CMP 4x3-------------------//
	Int FaceNumInRow = 4;
	Int FaceNumInColumn = 3;
	Int PixelNumInRowInFace_O = refPic->getWidth(COMPONENT_Y) / FaceNumInRow;
	Int PixelNumInColumnInFace_O = refPic->getHeight(COMPONENT_Y) / FaceNumInColumn;
	Int PixelNumInRowInFace = PixelNumInRowInFace_O * 4;
	Int PixelNumInColumnInFace = PixelNumInColumnInFace_O * 4;
	Int N = PixelNumInRowInFace / 2;

	Int CtuX = CtuRsAddr%CtuNumInRow;
	Int CtuY = CtuRsAddr / CtuNumInRow;
	Int PuRsAddr = g_auiZscanToRaster[cu->getZorderIdxInCtu() + partAddr];
	Int PuNumInRow = 16;
	Int PuX = PuRsAddr%PuNumInRow;
	Int PuY = PuRsAddr / PuNumInRow;
	Int LAX = 0; Int LAY = 0;
	Int RAX = 0; Int RAY = 0;
	Int LBX = 0; Int LBY = 0;
	Int RBX = 0; Int RBY = 0;
	Int MCX = 0; Int MCY = 0;
	//---------------------calculate coordinates of current PU in the picture--------------//
	//-----multiple coordinates of current PU in the picture with 4 to keep the accuracy------------//
	Int CurrentPUX = 4 * (CtuX * 64 + PuX * 4);
	Int CurrentPUY = 4 * (CtuY * 64 + PuY * 4);

	//Int mvX = mv->getHor() >> shiftHor;
	//Int mvY = mv->getVer() >> shiftVer;
	Int mvX = mv->getHor();
	Int mvY = mv->getVer();
	//---------------------calculate coordinates of reference block in the picture--------------//
	Int ReferenceX = CurrentPUX + mvX;
	Int ReferenceY = CurrentPUY + mvY;
	double ReferenceXSub = 0;
	double ReferenceYSub = 0;//Sub_coordinate of the extension
	double ReferenceXSubPro = 0;
	double ReferenceYSubPro = 0;
	double ReferenceXProjection = 0;
	double ReferenceYProjection = 0;//侧面坐标系
	//------------calculate the current PU belongs to which face in picture with the format of CMP4x3----------------------------// 
	Int FaceNum = 8;
	if (CurrentPUX / PixelNumInRowInFace == 0)
	{
		if (CurrentPUY / PixelNumInColumnInFace == 0)
		{
			FaceNum = 1;
		}
		else if (CurrentPUY / PixelNumInColumnInFace == 1)
		{
			FaceNum = 2;
		}
		else if (CurrentPUY / PixelNumInColumnInFace == 2)
		{
			FaceNum = 6;
		}
	}
	else if (CurrentPUX / PixelNumInRowInFace == 1)
	{
		if (CurrentPUY / PixelNumInColumnInFace == 0)
		{
			FaceNum = 0;//FaceNum=0, current CU belongs to no-data grey area
		}
		else if (CurrentPUY / PixelNumInColumnInFace == 1)
		{
			FaceNum = 3;
		}
		else if (CurrentPUY / PixelNumInColumnInFace == 2)
		{
			FaceNum = 0;
		}
	}
	else if (CurrentPUX / PixelNumInRowInFace == 2)
	{
		if (CurrentPUY / PixelNumInColumnInFace == 0)
		{
			FaceNum = 0;
		}
		else if (CurrentPUY / PixelNumInColumnInFace == 1)
		{
			FaceNum = 4;
		}
		else if (CurrentPUY / PixelNumInColumnInFace == 2)
		{
			FaceNum = 0;
		}
	}
	else
	{
		if (CurrentPUY / PixelNumInColumnInFace == 0)
		{
			FaceNum = 0;
		}
		else if (CurrentPUY / PixelNumInColumnInFace == 1)
		{
			FaceNum = 5;
		}
		else if (CurrentPUY / PixelNumInColumnInFace == 2)
		{
			FaceNum = 0;
		}
	}

	//------------------regular block motion compensation or sub-block projection-based motion compensation------------------------------------------------//
	if (CurrentPUX / PixelNumInRowInFace == ReferenceX / PixelNumInRowInFace && CurrentPUY / PixelNumInColumnInFace == ReferenceY / PixelNumInColumnInFace)
	{//-------------reference block and current CU belong to the same face,regular motion compensation----------------------------------------------------//
		xPredInterBlk(compID, cu, refPic, partAddr, mv, width, height, dstPic, bi, bitDepth);
	}//-------------reference block and current CU do not belong to the same face----------------------------------------------------//
	else
	{
		switch (FaceNum)
		{
		case 1:// current face belong to face 1
			//-------------------------------Get coordinates of four corners of face 1--------------------------------//
			LAX = 0; LAY = 0;
			RAX = 2 * N - 1; RAY = 0;
			LBX = 0; LBY = 2 * N - 1;
			RBX = 2 * N - 1; RBY = 2 * N - 1;
			MCX = (LAX + RAX + 1) / 2;
			MCY = (LAY + 1 + LBY) / 2;
			if (ReferenceX - RAX > 0 && abs(ReferenceX - MCX) > abs(ReferenceY - MCY)) //Area A
			{
				ReferenceXSub = ReferenceX - 2 * N;
				ReferenceYSub = N + 1 - ReferenceY;
				//---------------------------Projection Transform----------------------------------//
				ReferenceXSubPro = (N * ReferenceXSub) / (N + ReferenceXSub);
				ReferenceYSubPro = (N * ReferenceYSub) / (N + ReferenceXSub);
				//---------------------------Coordinates of Reference Block after modification in picture------------------//
				ReferenceXProjection = 3 * N + ReferenceYSubPro;
				ReferenceYProjection = 2 * N + ReferenceXSubPro - 1;
				RotationIdx = 1;
			}
			else if (ReferenceY - LAY <0 && abs(ReferenceX - MCX) < abs(ReferenceY - MCY))  //Area B
			{
				ReferenceXSub = 1 - ReferenceY;
				ReferenceYSub = N + 1 - ReferenceX;
				//---------------------------Projection Transform----------------------------------//
				ReferenceXSubPro = (N * ReferenceXSub) / (N + ReferenceXSub);
				ReferenceYSubPro = (N * ReferenceYSub) / (N + ReferenceXSub);
				//---------------------------Coordinates of Reference Block after modification in picture------------------//
				ReferenceXProjection = 5 * N + ReferenceXSubPro - 1;
				ReferenceYProjection = 2 * N + ReferenceYSubPro - 1;
				RotationIdx = 2;
			}
			else if (ReferenceX - LAX < 0 && abs(ReferenceX - MCX) > abs(ReferenceY - MCY))  //Area C
			{
				ReferenceXSub = 1 - ReferenceX;
				ReferenceYSub = ReferenceY - N;
				//---------------------------Projection Transform----------------------------------//
				ReferenceXSubPro = (N * ReferenceXSub) / (N + ReferenceXSub);
				ReferenceYSubPro = (N * ReferenceYSub) / (N + ReferenceXSub);
				//---------------------------Coordinates of Reference Block after modification in picture------------------//
				ReferenceXProjection = 7 * N - ReferenceYSubPro - 1;
				ReferenceYProjection = 2 * N + ReferenceXSubPro;
				RotationIdx = 3;
			}
			else  //Area D
			{
				ReferenceXSub = ReferenceY - 2 * N;
				ReferenceYSub = ReferenceX - N;
				//---------------------------Projection Transform----------------------------------//
				ReferenceXSubPro = (N * ReferenceXSub) / (N + ReferenceXSub);
				ReferenceYSubPro = (N * ReferenceYSub) / (N + ReferenceXSub);
				//---------------------------Coordinates of Reference Block after modification in picture------------------//
				ReferenceXProjection = N + ReferenceYSubPro;
				ReferenceYProjection = 2 * N + ReferenceXSubPro;
				RotationIdx = 0;
			}
			break;

		case 2:// current face belong to face 2
			//-------------------------------Get coordinates of four corners of face 2--------------------------------//
			LAX = 0; LAY = 2 * N;
			RAX = 2 * N - 1; RAY = 2 * N;
			LBX = 0; LBY = 4 * N - 1;
			RBX = 2 * N - 1; RBY = 4 * N - 1;
			MCX = (LAX + RAX + 1) / 2;
			MCY = (LAY + 1 + LBY) / 2;
			if (ReferenceX - RAX > 0 && abs(ReferenceX - MCX) > abs(ReferenceY - MCY)) //Area A
			{
				ReferenceXSub = ReferenceX - 2 * N;
				ReferenceYSub = 3 * N + 1 - ReferenceY;
				//---------------------------Projection Transform----------------------------------//
				ReferenceXSubPro = (N * ReferenceXSub) / (N + ReferenceXSub);
				ReferenceYSubPro = (N * ReferenceYSub) / (N + ReferenceXSub);
				//---------------------------Coordinates of Reference Block after modification in picture------------------//
				ReferenceXProjection = 2 * N + ReferenceXSubPro;
				ReferenceYProjection = 3 * N - ReferenceYSubPro - 1;
				RotationIdx = 0;
			}
			else if (ReferenceY - LAY <0 && abs(ReferenceX - MCX) < abs(ReferenceY - MCY))  //Area B
			{
				ReferenceXSub = 2 * N + 1 - ReferenceY;
				ReferenceYSub = N + 1 - ReferenceX;
				//---------------------------Projection Transform----------------------------------//
				ReferenceXSubPro = (N * ReferenceXSub) / (N + ReferenceXSub);
				ReferenceYSubPro = (N * ReferenceYSub) / (N + ReferenceXSub);
				//---------------------------Coordinates of Reference Block after modification in picture------------------//
				ReferenceXProjection = N - ReferenceYSubPro - 1;
				ReferenceYProjection = 2 * N - ReferenceXSubPro - 1;
				RotationIdx = 0;
			}
			else if (ReferenceX - LAX < 0 && abs(ReferenceX - MCX) > abs(ReferenceY - MCY))  //Area C
			{
				ReferenceXSub = 1 - ReferenceX;
				ReferenceYSub = ReferenceY - 3 * N;
				//---------------------------Projection Transform----------------------------------//
				ReferenceXSubPro = (N * ReferenceXSub) / (N + ReferenceXSub);
				ReferenceYSubPro = (N * ReferenceYSub) / (N + ReferenceXSub);
				//---------------------------Coordinates of Reference Block after modification in picture------------------//
				ReferenceXProjection = 8 * N - ReferenceXSubPro - 1;
				ReferenceYProjection = 3 * N + ReferenceYSubPro;
				RotationIdx = 0;
			}
			else  //Area D
			{
				ReferenceXSub = ReferenceY - 4 * N;
				ReferenceYSub = ReferenceX - N;
				//---------------------------Projection Transform----------------------------------//
				ReferenceXSubPro = (N * ReferenceXSub) / (N + ReferenceXSub);
				ReferenceYSubPro = (N * ReferenceYSub) / (N + ReferenceXSub);
				//---------------------------Coordinates of Reference Block after modification in picture------------------//
				ReferenceXProjection = N + ReferenceYSubPro;
				ReferenceYProjection = 2 * N + ReferenceXSubPro;
				RotationIdx = 0;
			}
			break;

		case 3:// current face belong to face 3
			//-------------------------------Get coordinates of four corners of face 3--------------------------------//
			LAX = 2 * N; LAY = 2 * N;
			RAX = 4 * N - 1; RAY = 2 * N;
			LBX = 2 * N; LBY = 4 * N - 1;
			RBX = 4 * N - 1; RBY = 4 * N - 1;
			MCX = (LAX + RAX + 1) / 2;
			MCY = (LAY + 1 + LBY) / 2;
			if (ReferenceX - RAX > 0 && abs(ReferenceX - MCX) > abs(ReferenceY - MCY)) //Area A
			{
				ReferenceXSub = ReferenceX - 4 * N;
				ReferenceYSub = 3 * N + 1 - ReferenceY;
				//---------------------------Projection Transform----------------------------------//
				ReferenceXSubPro = (N * ReferenceXSub) / (N + ReferenceXSub);
				ReferenceYSubPro = (N * ReferenceYSub) / (N + ReferenceXSub);
				//---------------------------Coordinates of Reference Block after modification in picture------------------//
				ReferenceXProjection = 4 * N + ReferenceXSubPro;
				ReferenceYProjection = 3 * N + ReferenceYSubPro - 1;
				RotationIdx = 0;
			}
			else if (ReferenceY - LAY <0 && abs(ReferenceX - MCX) < abs(ReferenceY - MCY))  //Area B
			{
				ReferenceXSub = 2 * N + 1 - ReferenceY;
				ReferenceYSub = 3 * N + 1 - ReferenceX;
				//---------------------------Projection Transform----------------------------------//
				ReferenceXSubPro = (N * ReferenceXSub) / (N + ReferenceXSub);
				ReferenceYSubPro = (N * ReferenceYSub) / (N + ReferenceXSub);
				//---------------------------Coordinates of Reference Block after modification in picture------------------//
				ReferenceXProjection = 2 * N - ReferenceXSubPro - 1;
				ReferenceYProjection = N + ReferenceYSubPro - 1;
				RotationIdx = 3;
			}
			else if (ReferenceX - LAX < 0 && abs(ReferenceX - MCX) > abs(ReferenceY - MCY))  //Area C
			{
				ReferenceXSub = 2 * N + 1 - ReferenceX;
				ReferenceYSub = ReferenceY - 3 * N;
				//---------------------------Projection Transform----------------------------------//
				ReferenceXSubPro = (N * ReferenceXSub) / (N + ReferenceXSub);
				ReferenceYSubPro = (N * ReferenceYSub) / (N + ReferenceXSub);
				//---------------------------Coordinates of Reference Block after modification in picture------------------//
				ReferenceXProjection = 2 * N - ReferenceXSubPro - 1;
				ReferenceYProjection = 3 * N + ReferenceYSubPro;
				RotationIdx = 0;
			}
			else  //Area D
			{
				ReferenceXSub = ReferenceY - 4 * N;
				ReferenceYSub = ReferenceX - 3 * N;
				//---------------------------Projection Transform----------------------------------//
				ReferenceXSubPro = (N * ReferenceXSub) / (N + ReferenceXSub);
				ReferenceYSubPro = (N * ReferenceYSub) / (N + ReferenceXSub);
				//---------------------------Coordinates of Reference Block after modification in picture------------------//
				ReferenceXProjection = 2 * N - ReferenceXSubPro;
				ReferenceYProjection = 5 * N + ReferenceYSubPro;
				RotationIdx = 1;
			}
			break;

		case 4:// current face belong to face 4
			//-------------------------------Get coordinates of four corners of face 4--------------------------------//
			LAX = 4 * N; LAY = 2 * N;
			RAX = 6 * N - 1; RAY = 2 * N;
			LBX = 4 * N; LBY = 4 * N - 1;
			RBX = 6 * N - 1; RBY = 4 * N - 1;
			MCX = (LAX + RAX + 1) / 2;
			MCY = (LAY + 1 + LBY) / 2;
			if (ReferenceX - RAX > 0 && abs(ReferenceX - MCX) > abs(ReferenceY - MCY)) //Area A
			{
				ReferenceXSub = ReferenceX - 6 * N;
				ReferenceYSub = 3 * N + 1 - ReferenceY;
				//---------------------------Projection Transform----------------------------------//
				ReferenceXSubPro = (N * ReferenceXSub) / (N + ReferenceXSub);
				ReferenceYSubPro = (N * ReferenceYSub) / (N + ReferenceXSub);
				//---------------------------Coordinates of Reference Block after modification in picture------------------//
				ReferenceXProjection = 6 * N + ReferenceXSubPro;
				ReferenceYProjection = 3 * N + ReferenceYSubPro - 1;
				RotationIdx = 0;
			}
			else if (ReferenceY - LAY <0 && abs(ReferenceX - MCX) < abs(ReferenceY - MCY))  //Area B
			{
				ReferenceXSub = 2 * N + 1 - ReferenceY;
				ReferenceYSub = 5 * N + 1 - ReferenceX;
				//---------------------------Projection Transform----------------------------------//
				ReferenceXSubPro = (N * ReferenceXSub) / (N + ReferenceXSub);
				ReferenceYSubPro = (N * ReferenceYSub) / (N + ReferenceXSub);
				//---------------------------Coordinates of Reference Block after modification in picture------------------//
				ReferenceXProjection = N + ReferenceYSubPro - 1;
				ReferenceYProjection = ReferenceXSubPro - 1;
				RotationIdx = 2;
			}
			else if (ReferenceX - LAX < 0 && abs(ReferenceX - MCX) > abs(ReferenceY - MCY))  //Area C
			{
				ReferenceXSub = 4 * N + 1 - ReferenceX;
				ReferenceYSub = ReferenceY - 3 * N;
				//---------------------------Projection Transform----------------------------------//
				ReferenceXSubPro = (N * ReferenceXSub) / (N + ReferenceXSub);
				ReferenceYSubPro = (N * ReferenceYSub) / (N + ReferenceXSub);
				//---------------------------Coordinates of Reference Block after modification in picture------------------//
				ReferenceXProjection = 4 * N - ReferenceXSubPro - 1;
				ReferenceYProjection = 3 * N + ReferenceYSubPro;
				RotationIdx = 0;
			}
			else  //Area D
			{
				ReferenceXSub = ReferenceY - 4 * N;
				ReferenceYSub = ReferenceX - 5 * N;
				//---------------------------Projection Transform----------------------------------//
				ReferenceXSubPro = (N * ReferenceXSub) / (N + ReferenceXSub);
				ReferenceYSubPro = (N * ReferenceYSub) / (N + ReferenceXSub);
				//--------------------------Coordinates of Reference Block after modification in picture------------------//
				ReferenceXProjection = N - ReferenceYSubPro;
				ReferenceYProjection = 6 * N - ReferenceXSubPro;
				RotationIdx = 2;
			}
			break;

		case 5:// current face belong to face 5
			//-------------------------------Get coordinates of four corners of face 5--------------------------------//
			LAX = 6 * N; LAY = 2 * N;
			RAX = 8 * N - 1; RAY = 2 * N;
			LBX = 6 * N; LBY = 4 * N - 1;
			RBX = 8 * N - 1; RBY = 4 * N - 1;
			MCX = (LAX + RAX + 1) / 2;
			MCY = (LAY + 1 + LBY) / 2;
			if (ReferenceX - RAX > 0 && abs(ReferenceX - MCX) > abs(ReferenceY - MCY)) //Area A
			{
				ReferenceXSub = ReferenceX - 8 * N;
				ReferenceYSub = 3 * N + 1 - ReferenceY;
				//---------------------------Projection Transform----------------------------------//
				ReferenceXSubPro = (N * ReferenceXSub) / (N + ReferenceXSub);
				ReferenceYSubPro = (N * ReferenceYSub) / (N + ReferenceXSub);
				//---------------------------Coordinates of Reference Block after modification in picture------------------//
				ReferenceXProjection = ReferenceXSubPro;
				ReferenceYProjection = 3 * N + ReferenceYSubPro - 1;
				RotationIdx = 0;
			}
			else if (ReferenceY - LAY <0 && abs(ReferenceX - MCX) < abs(ReferenceY - MCY))  //Area B
			{
				ReferenceXSub = 2 * N + 1 - ReferenceY;
				ReferenceYSub = 7 * N + 1 - ReferenceX;
				//---------------------------Projection Transform----------------------------------//
				ReferenceXSubPro = (N * ReferenceXSub) / (N + ReferenceXSub);
				ReferenceYSubPro = (N * ReferenceYSub) / (N + ReferenceXSub);
				//---------------------------Coordinates of Reference Block after modification in picture------------------//
				ReferenceXProjection = ReferenceXSubPro - 1;
				ReferenceYProjection = N - ReferenceYSubPro - 1;
				RotationIdx = 1;
			}
			else if (ReferenceX - LAX < 0 && abs(ReferenceX - MCX) > abs(ReferenceY - MCY))  //Area C
			{
				ReferenceXSub = 6 * N + 1 - ReferenceX;
				ReferenceYSub = ReferenceY - 3 * N;
				//---------------------------Projection Transform----------------------------------//
				ReferenceXSubPro = (N * ReferenceXSub) / (N + ReferenceXSub);
				ReferenceYSubPro = (N * ReferenceYSub) / (N + ReferenceXSub);
				//---------------------------Coordinates of Reference Block after modification in picture------------------//
				ReferenceXProjection = 6 * N - ReferenceXSubPro - 1;
				ReferenceYProjection = 3 * N + ReferenceYSubPro;
				RotationIdx = 0;
			}
			else  //Area D
			{
				ReferenceXSub = ReferenceY - 4 * N;
				ReferenceYSub = ReferenceX - 7 * N;
				//---------------------------Projection Transform----------------------------------//
				ReferenceXSubPro = (N * ReferenceXSub) / (N + ReferenceXSub);
				ReferenceYSubPro = (N * ReferenceYSub) / (N + ReferenceXSub);
				//---------------------------Coordinates of Reference Block after modification in picture------------------//
				ReferenceXProjection = ReferenceXSubPro;
				ReferenceYProjection = 5 * N - ReferenceYSubPro;
				RotationIdx = 3;
			}
			break;

		case 6:// current face belong to face 6
			//-------------------------------Get coordinates of four corners of face 6--------------------------------//
			LAX = 0; LAY = 4 * N;
			RAX = 2 * N - 1; RAY = 4 * N;
			LBX = 0; LBY = 6 * N - 1;
			RBX = 2 * N - 1; RBY = 6 * N - 1;
			MCX = (LAX + RAX + 1) / 2;
			MCY = (LAY + 1 + LBY) / 2;
			if (ReferenceX - RAX > 0 && abs(ReferenceX - MCX) > abs(ReferenceY - MCY)) //Area A
			{
				ReferenceXSub = ReferenceX - 2 * N;
				ReferenceYSub = 5 * N + 1 - ReferenceY;
				//---------------------------Projection Transform----------------------------------//
				ReferenceXSubPro = (N * ReferenceXSub) / (N + ReferenceXSub);
				ReferenceYSubPro = (N * ReferenceYSub) / (N + ReferenceXSub);
				//---------------------------Coordinates of Reference Block after modification in picture------------------//
				ReferenceXProjection = 3 * N - ReferenceYSubPro;
				ReferenceYProjection = 4 * N + ReferenceXSubPro - 1;
				RotationIdx = 3;
			}
			else if (ReferenceY - LAY <0 && abs(ReferenceX - MCX) < abs(ReferenceY - MCY))  //Area B
			{
				ReferenceXSub = 4 * N + 1 - ReferenceY;
				ReferenceYSub = N + 1 - ReferenceX;
				//---------------------------Projection Transform----------------------------------//
				ReferenceXSubPro = (N * ReferenceXSub) / (N + ReferenceXSub);
				ReferenceYSubPro = (N * ReferenceYSub) / (N + ReferenceXSub);
				//---------------------------Coordinates of Reference Block after modification in picture------------------//
				ReferenceXProjection = N - ReferenceYSubPro - 1;
				ReferenceYProjection = 4 * N - ReferenceXSubPro - 1;
				RotationIdx = 0;
			}
			else if (ReferenceX - LAX < 0 && abs(ReferenceX - MCX) > abs(ReferenceY - MCY))  //Area C
			{
				ReferenceXSub = 1 - ReferenceX;
				ReferenceYSub = ReferenceY - 5 * N;
				//---------------------------Projection Transform----------------------------------//
				ReferenceXSubPro = (N * ReferenceXSub) / (N + ReferenceXSub);
				ReferenceYSubPro = (N * ReferenceYSub) / (N + ReferenceXSub);
				//---------------------------Coordinates of Reference Block after modification in picture------------------//
				ReferenceXProjection = 7 * N - ReferenceYSubPro - 1;
				ReferenceYProjection = 4 * N - ReferenceXSubPro;
				RotationIdx = 1;
			}
			else  //Area D
			{
				ReferenceXSub = ReferenceY - 6 * N;
				ReferenceYSub = ReferenceX - N;
				//---------------------------Projection Transform----------------------------------//
				ReferenceXSubPro = (N * ReferenceXSub) / (N + ReferenceXSub);
				ReferenceYSubPro = (N * ReferenceYSub) / (N + ReferenceXSub);
				//---------------------------Coordinates of Reference Block after modification in picture------------------//
				ReferenceXProjection = 5 * N - ReferenceYSubPro;
				ReferenceYProjection = 4 * N - ReferenceXSubPro;
				RotationIdx = 2;
			}
			break;
		case 0:
			ReferenceXProjection = ReferenceX;
			ReferenceYProjection = ReferenceY;
			break;
		case 8:
		{
			cerr << "FaceNum not assigned.2/n" << endl;
		}
			break;
		default:
		{
			cerr << "FaceNum = " << FaceNum << "/n" << endl;
		}
			break;
		}
		Int mvX_2 = ReferenceXProjection - CurrentPUX;
		Int mvY_2 = ReferenceYProjection - CurrentPUY;
		TComMv mv1(0, 0);
		mv1.setHor(mvX_2);
		mv1.setVer(mvY_2);
		xPredInterBlk_PBSMC(compID, cu, refPic, partAddr, mv1, width, height, dstPic, bi, bitDepth, RotationIdx);

	}
}
#endif
#endif
/**
* \calculate the coordinates on the sub-coordinate of the extension part of the current face according to the coordinates of the reference block
* \CMP4x3 format
*
* \param compID     Colour component ID
* \param cu         Pointer to current CU
* \param refPic     Pointer to reference picture
* \param partAddr   Address of block within CU
* \param mv         Motion vector
* \param width      Width of block
* \param height     Height of block
* \param dstPic     Pointer to destination picture
* \param bi         Flag indicating whether bipred is used
* \param  bitDepth  Bit depth
*/

#if ZhengRuidi_20170315
Void TComPrediction::xPredInterBlk_PBSMC(const ComponentID compID, TComDataCU *cu, TComPicYuv *refPic, UInt partAddr, TComMv mv, Int width, Int height, TComYuv *dstPic, Bool bi, const Int bitDepth, Int RotationIdx)
{
	Int     refStride = refPic->getStride(compID);
	Int     dstStride = dstPic->getStride(compID);
	Int shiftHor = (2 + refPic->getComponentScaleX(compID));
	Int shiftVer = (2 + refPic->getComponentScaleY(compID));

	Int     refOffset = (mv.getHor() >> shiftHor) + (mv.getVer() >> shiftVer) * refStride;

	// Pel*    cur = refPic->getAddr(compID, cu->getCtuRsAddr(), cu->getZorderIdxInCtu() + partAddr);
	Pel*    ref = refPic->getAddr(compID, cu->getCtuRsAddr(), cu->getZorderIdxInCtu() + partAddr) + refOffset;
	Pel*    dst = dstPic->getAddr(compID, partAddr);
	Int     xFrac = mv.getHor() & ((1 << shiftHor) - 1);  //取1/4亚像素精度的MV的水平分量的最低的两位，也就是整像素精度的MV小数点后
	Int     yFrac = mv.getVer() & ((1 << shiftVer) - 1);  //取1/4亚像素精度的MV的竖直分量的最低的两位，也就是整像素精度的MV小数点后
	UInt    cxWidth = width >> refPic->getComponentScaleX(compID);
	UInt    cxHeight = height >> refPic->getComponentScaleY(compID);

	const ChromaFormat chFmt = cu->getPic()->getChromaFormat();
	//--------------------modify pixels in reference block to get right compensation value with the format of CMP4x3---------------------------//
	if (yFrac == 0)
	{
#if ZhengRuidi_20170315
		m_if.filterHor_P(compID, ref, refStride, dst, dstStride, cxWidth, cxHeight, xFrac, !bi, chFmt, bitDepth, RotationIdx);
#else
		m_if.filterHor(compID, ref, refStride, dst, dstStride, cxWidth, cxHeight, xFrac, !bi, chFmt, bitDepth);
#endif // ZhengRuidi_20170315
	}
	else if (xFrac == 0)
	{
#if ZhengRuidi_20170315
		m_if.filterVer_P(compID, ref, refStride, dst, dstStride, cxWidth, cxHeight, yFrac, true, !bi, chFmt, bitDepth, RotationIdx);
#else // ZhengRuidi_20170315
		m_if.filterVer(compID, ref, refStride, dst, dstStride, cxWidth, cxHeight, yFrac, true, !bi, chFmt, bitDepth);
#endif // ZhengRuidi_20170315

	}
	else
	{
		Int   tmpStride = m_filteredBlockTmp[0].getStride(compID);
		Pel*  tmp = m_filteredBlockTmp[0].getAddr(compID);

		const Int vFilterSize = isLuma(compID) ? NTAPS_LUMA : NTAPS_CHROMA;

#if ZhengRuidi_20170315
		m_if.filterHor_P(compID, ref - ((vFilterSize >> 1) - 1)*refStride, refStride, tmp, tmpStride, cxWidth, cxHeight + vFilterSize - 1, xFrac, false, chFmt, bitDepth, RotationIdx);
		m_if.filterVer_P(compID, tmp + ((vFilterSize >> 1) - 1)*tmpStride, tmpStride, dst, dstStride, cxWidth, cxHeight, yFrac, false, !bi, chFmt, bitDepth, RotationIdx);
#else // ZhengRuidi_20170315
		m_if.filterHor(compID, ref - ((vFilterSize >> 1) - 1)*refStride, refStride, tmp, tmpStride, cxWidth, cxHeight + vFilterSize - 1, xFrac, false, chFmt, bitDepth);
		m_if.filterVer(compID, tmp + ((vFilterSize >> 1) - 1)*tmpStride, tmpStride, dst, dstStride, cxWidth, cxHeight, yFrac, false, !bi, chFmt, bitDepth);
#endif // ZhengRuidi_20170315

	}
}
#endif

/**
* \brief Generate motion-compensated block using PBSMC
*
* \param compID     Colour component ID
* \param cu         Pointer to current CU
* \param refPic     Pointer to reference picture
* \param partAddr   Address of block within CU
* \param mv         Motion vector
* \param width      Width of block
* \param height     Height of block
* \param dstPic     Pointer to destination picture
* \param bi         Flag indicating whether bipred is used
* \param  bitDepth  Bit depth
*/

/**
 * \brief Generate motion-compensated block
 *
 * \param compID     Colour component ID
 * \param cu         Pointer to current CU
 * \param refPic     Pointer to reference picture
 * \param partAddr   Address of block within CU
 * \param mv         Motion vector
 * \param width      Width of block
 * \param height     Height of block
 * \param dstPic     Pointer to destination picture
 * \param bi         Flag indicating whether bipred is used
 * \param  bitDepth  Bit depth
 */


Void TComPrediction::xPredInterBlk(const ComponentID compID, TComDataCU *cu, TComPicYuv *refPic, UInt partAddr, TComMv *mv, Int width, Int height, TComYuv *dstPic, Bool bi, const Int bitDepth )
{
  Int     refStride  = refPic->getStride(compID);
  Int     dstStride  = dstPic->getStride(compID);
  Int shiftHor=(2+refPic->getComponentScaleX(compID));
  Int shiftVer=(2+refPic->getComponentScaleY(compID));

  Int     refOffset  = (mv->getHor() >> shiftHor) + (mv->getVer() >> shiftVer) * refStride;

  Pel*    ref     = refPic->getAddr(compID, cu->getCtuRsAddr(), cu->getZorderIdxInCtu() + partAddr ) + refOffset;

  Pel*    dst = dstPic->getAddr( compID, partAddr );

  Int     xFrac  = mv->getHor() & ((1<<shiftHor)-1);  //取1/4亚像素精度的MV的水平分量的最低的两位，也就是整像素精度的MV小数点后，对于色度取的是后三位精度
  Int     yFrac  = mv->getVer() & ((1<<shiftVer)-1);  //取1/4亚像素精度的MV的竖直分量的最低的两位，也就是整像素精度的MV小数点后
  UInt    cxWidth  = width  >> refPic->getComponentScaleX(compID); 
  UInt    cxHeight = height >> refPic->getComponentScaleY(compID);

  const ChromaFormat chFmt = cu->getPic()->getChromaFormat();

  if ( yFrac == 0 )
  {
    m_if.filterHor(compID, ref, refStride, dst,  dstStride, cxWidth, cxHeight, xFrac, !bi, chFmt, bitDepth);
  }
  else if ( xFrac == 0 )
  {
    m_if.filterVer(compID, ref, refStride, dst, dstStride, cxWidth, cxHeight, yFrac, true, !bi, chFmt, bitDepth);
  }
  else

  {
    Int   tmpStride = m_filteredBlockTmp[0].getStride(compID);
    Pel*  tmp       = m_filteredBlockTmp[0].getAddr(compID);

    const Int vFilterSize = isLuma(compID) ? NTAPS_LUMA : NTAPS_CHROMA;

    m_if.filterHor(compID, ref - ((vFilterSize>>1) -1)*refStride, refStride, tmp, tmpStride, cxWidth, cxHeight+vFilterSize-1, xFrac, false,      chFmt, bitDepth);
    m_if.filterVer(compID, tmp + ((vFilterSize>>1) -1)*tmpStride, tmpStride, dst, dstStride, cxWidth, cxHeight,               yFrac, false, !bi, chFmt, bitDepth);
  }
}

Void TComPrediction::xWeightedAverage( TComYuv* pcYuvSrc0, TComYuv* pcYuvSrc1, Int iRefIdx0, Int iRefIdx1, UInt uiPartIdx, Int iWidth, Int iHeight, TComYuv* pcYuvDst, const BitDepths &clipBitDepths )
{
  if( iRefIdx0 >= 0 && iRefIdx1 >= 0 )
  {
    pcYuvDst->addAvg( pcYuvSrc0, pcYuvSrc1, uiPartIdx, iWidth, iHeight, clipBitDepths );
  }
  else if ( iRefIdx0 >= 0 && iRefIdx1 <  0 )
  {
    pcYuvSrc0->copyPartToPartYuv( pcYuvDst, uiPartIdx, iWidth, iHeight );
  }
  else if ( iRefIdx0 <  0 && iRefIdx1 >= 0 )
  {
    pcYuvSrc1->copyPartToPartYuv( pcYuvDst, uiPartIdx, iWidth, iHeight );
  }
}

// AMVP
Void TComPrediction::getMvPredAMVP( TComDataCU* pcCU, UInt uiPartIdx, UInt uiPartAddr, RefPicList eRefPicList, TComMv& rcMvPred )
{
  AMVPInfo* pcAMVPInfo = pcCU->getCUMvField(eRefPicList)->getAMVPInfo();

  if( pcAMVPInfo->iN <= 1 )
  {
    rcMvPred = pcAMVPInfo->m_acMvCand[0];

    pcCU->setMVPIdxSubParts( 0, eRefPicList, uiPartAddr, uiPartIdx, pcCU->getDepth(uiPartAddr));
    pcCU->setMVPNumSubParts( pcAMVPInfo->iN, eRefPicList, uiPartAddr, uiPartIdx, pcCU->getDepth(uiPartAddr));
    return;
  }

  assert(pcCU->getMVPIdx(eRefPicList,uiPartAddr) >= 0);
  rcMvPred = pcAMVPInfo->m_acMvCand[pcCU->getMVPIdx(eRefPicList,uiPartAddr)];
  return;
}

/** Function for deriving planar intra prediction.
 * \param pSrc        pointer to reconstructed sample array
 * \param srcStride   the stride of the reconstructed sample array
 * \param rpDst       reference to pointer for the prediction sample array
 * \param dstStride   the stride of the prediction sample array
 * \param width       the width of the block
 * \param height      the height of the block
 * \param channelType type of pel array (luma, chroma)
 * \param format      chroma format
 *
 * This function derives the prediction samples for planar mode (intra coding).
 */
//NOTE: Bit-Limit - 24-bit source
Void TComPrediction::xPredIntraPlanar( const Pel* pSrc, Int srcStride, Pel* rpDst, Int dstStride, UInt width, UInt height )
{
  assert(width <= height);

  Int leftColumn[MAX_CU_SIZE+1], topRow[MAX_CU_SIZE+1], bottomRow[MAX_CU_SIZE], rightColumn[MAX_CU_SIZE];
  UInt shift1Dhor = g_aucConvertToBit[ width ] + 2;
  UInt shift1Dver = g_aucConvertToBit[ height ] + 2;

  // Get left and above reference column and row
  for(Int k=0;k<width+1;k++)
  {
    topRow[k] = pSrc[k-srcStride];
  }

  for (Int k=0; k < height+1; k++)
  {
    leftColumn[k] = pSrc[k*srcStride-1];
  }

  // Prepare intermediate variables used in interpolation
  Int bottomLeft = leftColumn[height];
  Int topRight   = topRow[width];

  for(Int k=0;k<width;k++)
  {
    bottomRow[k]  = bottomLeft - topRow[k];
    topRow[k]     <<= shift1Dver;
  }

  for(Int k=0;k<height;k++)
  {
    rightColumn[k]  = topRight - leftColumn[k];
    leftColumn[k]   <<= shift1Dhor;
  }

  const UInt topRowShift = 0;

  // Generate prediction signal
  for (Int y=0;y<height;y++)
  {
    Int horPred = leftColumn[y] + width;
    for (Int x=0;x<width;x++)
    {
      horPred += rightColumn[y];
      topRow[x] += bottomRow[x];

      Int vertPred = ((topRow[x] + topRowShift)>>topRowShift);
      rpDst[y*dstStride+x] = ( horPred + vertPred ) >> (shift1Dhor+1);
    }
  }
}

/** Function for filtering intra DC predictor.
 * \param pSrc pointer to reconstructed sample array
 * \param iSrcStride the stride of the reconstructed sample array
 * \param pDst reference to pointer for the prediction sample array
 * \param iDstStride the stride of the prediction sample array
 * \param iWidth the width of the block
 * \param iHeight the height of the block
 * \param channelType type of pel array (luma, chroma)
 *
 * This function performs filtering left and top edges of the prediction samples for DC mode (intra coding).
 */
Void TComPrediction::xDCPredFiltering( const Pel* pSrc, Int iSrcStride, Pel* pDst, Int iDstStride, Int iWidth, Int iHeight, ChannelType channelType )
{
  Int x, y, iDstStride2, iSrcStride2;

  if (isLuma(channelType) && (iWidth <= MAXIMUM_INTRA_FILTERED_WIDTH) && (iHeight <= MAXIMUM_INTRA_FILTERED_HEIGHT))
  {
    //top-left
    pDst[0] = (Pel)((pSrc[-iSrcStride] + pSrc[-1] + 2 * pDst[0] + 2) >> 2);

    //top row (vertical filter)
    for ( x = 1; x < iWidth; x++ )
    {
      pDst[x] = (Pel)((pSrc[x - iSrcStride] +  3 * pDst[x] + 2) >> 2);
    }

    //left column (horizontal filter)
    for ( y = 1, iDstStride2 = iDstStride, iSrcStride2 = iSrcStride-1; y < iHeight; y++, iDstStride2+=iDstStride, iSrcStride2+=iSrcStride )
    {
      pDst[iDstStride2] = (Pel)((pSrc[iSrcStride2] + 3 * pDst[iDstStride2] + 2) >> 2);
    }
  }

  return;
}

/* Static member function */
Bool TComPrediction::UseDPCMForFirstPassIntraEstimation(TComTU &rTu, const UInt uiDirMode)
{
  return (rTu.getCU()->isRDPCMEnabled(rTu.GetAbsPartIdxTU()) ) &&
          rTu.getCU()->getCUTransquantBypass(rTu.GetAbsPartIdxTU()) &&
          (uiDirMode==HOR_IDX || uiDirMode==VER_IDX);
}

//! \}
