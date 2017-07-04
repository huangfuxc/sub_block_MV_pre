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

/** \file     TDecSlice.cpp
    \brief    slice decoder class
*/

#include "TDecSlice.h"

//! \ingroup TLibDecoder
//! \{

//////////////////////////////////////////////////////////////////////
// Construction/Destruction
//////////////////////////////////////////////////////////////////////

TDecSlice::TDecSlice()
{
}

TDecSlice::~TDecSlice()
{
}

Void TDecSlice::create()
{
}

Void TDecSlice::destroy()
{
}

Void TDecSlice::init(TDecEntropy* pcEntropyDecoder, TDecCu* pcCuDecoder)
{
  m_pcEntropyDecoder  = pcEntropyDecoder;
  m_pcCuDecoder       = pcCuDecoder;
}

Void TDecSlice::decompressSlice(TComInputBitstream** ppcSubstreams, TComPic* pcPic, TDecSbac* pcSbacDecoder)
{
#if AMVP_DEBUG	
	FILE *Mvv0;
	Mvv0 = fopen("E:\\github\\sub_block_MV_pre\\HM-16.14_add\\cfg\\Mvv1.txt", "a");
	int count = 0;
#endif
  TComSlice* pcSlice                 = pcPic->getSlice(pcPic->getCurrSliceIdx());

  const Int  startCtuTsAddr          = pcSlice->getSliceSegmentCurStartCtuTsAddr();
  const Int  startCtuRsAddr          = pcPic->getPicSym()->getCtuTsToRsAddrMap(startCtuTsAddr);
  const UInt numCtusInFrame          = pcPic->getNumberOfCtusInFrame();

  const UInt frameWidthInCtus        = pcPic->getPicSym()->getFrameWidthInCtus();
  const Bool depSliceSegmentsEnabled = pcSlice->getPPS()->getDependentSliceSegmentsEnabledFlag();
  const Bool wavefrontsEnabled       = pcSlice->getPPS()->getEntropyCodingSyncEnabledFlag();

  m_pcEntropyDecoder->setEntropyDecoder ( pcSbacDecoder  );
  m_pcEntropyDecoder->setBitstream      ( ppcSubstreams[0] );
  m_pcEntropyDecoder->resetEntropy      (pcSlice);

  // decoder doesn't need prediction & residual frame buffer
  pcPic->setPicYuvPred( 0 );
  pcPic->setPicYuvResi( 0 );

#if ENC_DEC_TRACE
  g_bJustDoIt = g_bEncDecTraceEnable;
#endif
  DTRACE_CABAC_VL( g_nSymbolCounter++ );
  DTRACE_CABAC_T( "\tPOC: " );
  DTRACE_CABAC_V( pcPic->getPOC() );
  DTRACE_CABAC_T( "\n" );

#if ENC_DEC_TRACE
  g_bJustDoIt = g_bEncDecTraceDisable;
#endif

  // The first CTU of the slice is the first coded substream, but the global substream number, as calculated by getSubstreamForCtuAddr may be higher.
  // This calculates the common offset for all substreams in this slice.
  const UInt subStreamOffset=pcPic->getSubstreamForCtuAddr(startCtuRsAddr, true, pcSlice);


  if (depSliceSegmentsEnabled)
  {
    // modify initial contexts with previous slice segment if this is a dependent slice.
    const UInt startTileIdx=pcPic->getPicSym()->getTileIdxMap(startCtuRsAddr);
    const TComTile *pCurrentTile=pcPic->getPicSym()->getTComTile(startTileIdx);
    const UInt firstCtuRsAddrOfTile = pCurrentTile->getFirstCtuRsAddr();

    if( pcSlice->getDependentSliceSegmentFlag() && startCtuRsAddr != firstCtuRsAddrOfTile)
    {
      if ( pCurrentTile->getTileWidthInCtus() >= 2 || !wavefrontsEnabled)
      {
        pcSbacDecoder->loadContexts(&m_lastSliceSegmentEndContextState);
      }
    }
  }

  // for every CTU in the slice segment...

  Bool isLastCtuOfSliceSegment = false;
  for( UInt ctuTsAddr = startCtuTsAddr; !isLastCtuOfSliceSegment && ctuTsAddr < numCtusInFrame; ctuTsAddr++)
  {
    const UInt ctuRsAddr = pcPic->getPicSym()->getCtuTsToRsAddrMap(ctuTsAddr);
    const TComTile &currentTile = *(pcPic->getPicSym()->getTComTile(pcPic->getPicSym()->getTileIdxMap(ctuRsAddr)));
    const UInt firstCtuRsAddrOfTile = currentTile.getFirstCtuRsAddr();
    const UInt tileXPosInCtus = firstCtuRsAddrOfTile % frameWidthInCtus;
    const UInt tileYPosInCtus = firstCtuRsAddrOfTile / frameWidthInCtus;
    const UInt ctuXPosInCtus  = ctuRsAddr % frameWidthInCtus;
    const UInt ctuYPosInCtus  = ctuRsAddr / frameWidthInCtus;
    const UInt uiSubStrm=pcPic->getSubstreamForCtuAddr(ctuRsAddr, true, pcSlice)-subStreamOffset;
    TComDataCU* pCtu = pcPic->getCtu( ctuRsAddr );
    pCtu->initCtu( pcPic, ctuRsAddr );

    m_pcEntropyDecoder->setBitstream( ppcSubstreams[uiSubStrm] );

    // set up CABAC contexts' state for this CTU
    if (ctuRsAddr == firstCtuRsAddrOfTile)
    {
      if (ctuTsAddr != startCtuTsAddr) // if it is the first CTU, then the entropy coder has already been reset
      {
        m_pcEntropyDecoder->resetEntropy(pcSlice);
      }
    }
    else if (ctuXPosInCtus == tileXPosInCtus && wavefrontsEnabled)
    {
      // Synchronize cabac probabilities with upper-right CTU if it's available and at the start of a line.
      if (ctuTsAddr != startCtuTsAddr) // if it is the first CTU, then the entropy coder has already been reset
      {
        m_pcEntropyDecoder->resetEntropy(pcSlice);
      }
      TComDataCU *pCtuUp = pCtu->getCtuAbove();
      if ( pCtuUp && ((ctuRsAddr%frameWidthInCtus+1) < frameWidthInCtus)  )
      {
        TComDataCU *pCtuTR = pcPic->getCtu( ctuRsAddr - frameWidthInCtus + 1 );
        if ( pCtu->CUIsFromSameSliceAndTile(pCtuTR) )
        {
          // Top-right is available, so use it.
          pcSbacDecoder->loadContexts( &m_entropyCodingSyncContextState );
        }
      }
    }

#if ENC_DEC_TRACE
    g_bJustDoIt = g_bEncDecTraceEnable;
#endif

    if ( pcSlice->getSPS()->getUseSAO() )
    {
      SAOBlkParam& saoblkParam = (pcPic->getPicSym()->getSAOBlkParam())[ctuRsAddr];
      Bool bIsSAOSliceEnabled = false;
      Bool sliceEnabled[MAX_NUM_COMPONENT];
      for(Int comp=0; comp < MAX_NUM_COMPONENT; comp++)
      {
        ComponentID compId=ComponentID(comp);
        sliceEnabled[compId] = pcSlice->getSaoEnabledFlag(toChannelType(compId)) && (comp < pcPic->getNumberValidComponents());
        if (sliceEnabled[compId])
        {
          bIsSAOSliceEnabled=true;
        }
        saoblkParam[compId].modeIdc = SAO_MODE_OFF;
      }
      if (bIsSAOSliceEnabled)
      {
        Bool leftMergeAvail = false;
        Bool aboveMergeAvail= false;

        //merge left condition
        Int rx = (ctuRsAddr % frameWidthInCtus);
        if(rx > 0)
        {
          leftMergeAvail = pcPic->getSAOMergeAvailability(ctuRsAddr, ctuRsAddr-1);
        }
        //merge up condition
        Int ry = (ctuRsAddr / frameWidthInCtus);
        if(ry > 0)
        {
          aboveMergeAvail = pcPic->getSAOMergeAvailability(ctuRsAddr, ctuRsAddr-frameWidthInCtus);
        }

        pcSbacDecoder->parseSAOBlkParam( saoblkParam, sliceEnabled, leftMergeAvail, aboveMergeAvail, pcSlice->getSPS()->getBitDepths());
      }
    }

    m_pcCuDecoder->decodeCtu     ( pCtu, isLastCtuOfSliceSegment );
    m_pcCuDecoder->decompressCtu ( pCtu );

#if AMVP_DEBUG
	//TComMv MV0, MV1, MV2, MV3;
	//if (pCtu->getSlice()->getPOC() == 5)
	//{
	//	//	if (pCtu->getCtuRsAddr() == 41)
	//	//	{
	//	//		TComDataCU CU0, CU1;
	//	//		TComDataCU*m_ppcCU0 = &CU0;
	//	//		TComDataCU*m_ppcCU1 = &CU1;
	//	//	//	m_ppcCU0->initSubCU(pCtu, 0, 2, 22);
	//	//		m_ppcCU0->copyInterPredInfoFrom(pCtu, 0, REF_PIC_LIST_0);
	//	//		m_ppcCU0->copyInterPredInfoFrom(pCtu, 0, REF_PIC_LIST_1);//此处的uiAbsPartIdx是真正的一个CTU内部的Zorder顺序；
	//	//		//m_ppcCU1->copyInterPredInfoFrom(pCtu, 16, REF_PIC_LIST_0);
	//	//		//m_ppcCU1->copyInterPredInfoFrom(pCtu, 16, REF_PIC_LIST_1);//此处的uiAbsPartIdx是真正的一个CTU内部的Zorder顺序；
	//	//		//UInt uiAbsPartIdx = 0;
	//	//		//Int iWidth = 0;
	//	//		//Int iHeight = 0;
	//	//		//m_ppcCU0->getPartIndexAndSize(0, uiAbsPartIdx, iWidth, iHeight);
	//	//		//TComMvField cMvFieldNeighbours[MRG_MAX_NUM_CANDS << 1]; // double length for mv of both lists
	//	//		//UChar uhInterDirNeighbours[MRG_MAX_NUM_CANDS];
	//	//		//Int numValidMergeCand = 0;
	//	//		int iRefIdx = -1;
	//	//		//m_ppcCU0->getInterMergeCandidates(uiAbsPartIdx, 0, cMvFieldNeighbours, uhInterDirNeighbours, numValidMergeCand);
	//	//		//for (int uiRefListIdx = 0; uiRefListIdx < 2; uiRefListIdx++)
	//	//		//{
	//	//		//	/*if (!m_ppcCU0->getZorderIdxInCtu())
	//	//		//	{
	//	//		//		for (int i = 0; i < numValidMergeCand; i++)
	//	//		//			cout << " merge:  " << cMvFieldNeighbours[2 * i + uiRefListIdx].getMv().getHor() << "  " << cMvFieldNeighbours[2 * i + uiRefListIdx].getMv().getVer() << "  " << cMvFieldNeighbours[2 * i + uiRefListIdx].getMv().getPos() << endl;
	//	//		//	}
	//	//		//	*/
	//	//		//	TComCUMvField* pcSubCUMvField = m_ppcCU0->getCUMvField(RefPicList(uiRefListIdx));
	//	//		//	AMVPInfo* pAMVPInfo = pcSubCUMvField->getAMVPInfo();
	//	//		//	iRefIdx = pcSubCUMvField->getRefIdx(0);
	//	//		//	m_ppcCU0->fillMvpCand(0, 0, RefPicList(uiRefListIdx), iRefIdx, pAMVPInfo);
	//	//		//	if (m_ppcCU0->getZorderIdxInCtu() == 0 )
	//	//		//	{
	//	//		//		for (int i = 0; i < pAMVPInfo->iN; i++)
	//	//		//		{
	//	//		//			cout << pAMVPInfo->m_acMvCand[i].getHor() << " " << pAMVPInfo->m_acMvCand[i].getVer() << endl;
	//	//		//		}
	//	//		//	}
	//	//		//}
	//	//	}
	//	//		
	//	for (int i = 0; i < 256; i++)
	//	{
	//		MV0 = pCtu->getCUMvField(REF_PIC_LIST_0)->getMv(g_auiRasterToZscan[i]);
	//		MV1 = pCtu->getCUMvField(REF_PIC_LIST_1)->getMv(g_auiRasterToZscan[i]);
	//		MV2 = pCtu->getCUMvField(REF_PIC_LIST_0)->getMvd(g_auiRasterToZscan[i]);
	//		MV3 = pCtu->getCUMvField(REF_PIC_LIST_1)->getMvd(g_auiRasterToZscan[i]);
	//		count++;
	//		/*if (count == 11665||count==11666)
	//		{
	//			fprintf(Mvv0, "%4d", pCtu->getMVPIdx(REF_PIC_LIST_0, g_auiRasterToZscan[i]));
	//			fprintf(Mvv0, "%4d",  g_auiRasterToZscan[i]);
	//		}*/
	//		/*	if (count == 13825)
	//		{
	//		fprintf(Mvv0, "%4d", pCtu->getMergeIndex(g_auiRasterToZscan[i]));
	//		}*/
	//		//if (count == 10497)
	//		//		{
	//		//			//fprintf(Mvv0, "%4d", pCtu->getMergeFlag(g_auiRasterToZscan[i]));
	//		//			fprintf(Mvv0, "%4d", pCtu->getPartitionSize(g_auiRasterToZscan[i]));
	//		//			fprintf(Mvv0, "%4d", pCtu->getSlice()->getRefPOC(REF_PIC_LIST_0, pCtu->getCUMvField(REF_PIC_LIST_0)->getRefIdx(g_auiRasterToZscan[i])));
	//		////			fprintf(Mvv0, "%4d", pCtu->getMVPIdx(REF_PIC_LIST_0, g_auiRasterToZscan[i]));
	//		////		}
	//		//if (count == 7169)
	//		//{
	//		//	fprintf(Mvv0, "%4d", pCtu->getMVPIdx(REF_PIC_LIST_0, g_auiRasterToZscan[i]));
	//		//	fprintf(Mvv0, "%4d", pCtu->getSlice()->getRefPOC(REF_PIC_LIST_0, pCtu->getCUMvField(REF_PIC_LIST_0)->getRefIdx(g_auiRasterToZscan[i])));
	//		//}
	//		fprintf(Mvv0, "%4d%4d%4d%4d", MV0.getHor(), MV0.getVer(), MV2.getHor(), MV2.getVer());//水平与垂直方向上的位移
	//		//		//fprintf(Mvv0, "%4d%4d", MV2.getHor(), MV2.getVer());//水平与垂直方向上的位移
	//		//		if (count == 10497)
	//		//		{
	//		//			fprintf(Mvv0, "%4d", pCtu->getSlice()->getRefPOC(REF_PIC_LIST_1, pCtu->getCUMvField(REF_PIC_LIST_1)->getRefIdx(g_auiRasterToZscan[i])));
	//		//			fprintf(Mvv0, "%4d", pCtu->getMVPIdx(REF_PIC_LIST_1, g_auiRasterToZscan[i]));
	//		//		}
	//		//if (count == 7169)
	//		//	fprintf(Mvv0, "%4d", pCtu->getMVPIdx(REF_PIC_LIST_1, g_auiRasterToZscan[i]));
	//		fprintf(Mvv0, "%4d%4d%4d%4d\n", MV1.getHor(), MV1.getVer(), MV3.getHor(), MV3.getVer());//水平与垂直方向上的位移
	//		//		//fprintf(Mvv0, "%4d%4d\n", MV3.getHor(), MV3.getVer());//水平与垂直方向上的位移
	//		//	}
	//		//	
	//	}
	//}
#endif

#if ENC_DEC_TRACE
    g_bJustDoIt = g_bEncDecTraceDisable;
#endif

    //Store probabilities of second CTU in line into buffer
    if ( ctuXPosInCtus == tileXPosInCtus+1 && wavefrontsEnabled)
    {
      m_entropyCodingSyncContextState.loadContexts( pcSbacDecoder );
    }

    if (isLastCtuOfSliceSegment)
    {
#if DECODER_CHECK_SUBSTREAM_AND_SLICE_TRAILING_BYTES
      pcSbacDecoder->parseRemainingBytes(false);
#endif
      if(!pcSlice->getDependentSliceSegmentFlag())
      {
        pcSlice->setSliceCurEndCtuTsAddr( ctuTsAddr+1 );
      }
      pcSlice->setSliceSegmentCurEndCtuTsAddr( ctuTsAddr+1 );
    }
    else if (  ctuXPosInCtus + 1 == tileXPosInCtus + currentTile.getTileWidthInCtus() &&
             ( ctuYPosInCtus + 1 == tileYPosInCtus + currentTile.getTileHeightInCtus() || wavefrontsEnabled)
            )
    {
      // The sub-stream/stream should be terminated after this CTU.
      // (end of slice-segment, end of tile, end of wavefront-CTU-row)
      UInt binVal;
      pcSbacDecoder->parseTerminatingBit( binVal );
      assert( binVal );
#if DECODER_CHECK_SUBSTREAM_AND_SLICE_TRAILING_BYTES
      pcSbacDecoder->parseRemainingBytes(true);
#endif
    }

  }

  assert(isLastCtuOfSliceSegment == true);


  if( depSliceSegmentsEnabled )
  {
    m_lastSliceSegmentEndContextState.loadContexts( pcSbacDecoder );//ctx end of dep.slice
  }
#if AMVP_DEBUG
  fclose(Mvv0);
#endif
}

//! \}
