// Copyright 2014-2017 Oxford University Innovation Limited and the authors of InfiniTAM

#pragma once

#include "../../../Objects/Scene/ITMRepresentationAccess.h"
#include "../../../Utils/ITMPixelUtils.h"

template<class TVoxel>
_CPU_AND_GPU_CODE_ inline float computeUpdatedVoxelDepthInfo(DEVICEPTR(TVoxel) &voxel, const THREADPTR(Vector4f) & pt_model, const CONSTPTR(Matrix4f) & M_d,
	const CONSTPTR(Vector4f) & projParams_d, float mu, int maxW, const CONSTPTR(float) *depth, const CONSTPTR(Vector2i) & imgSize, const CONSTPTR(uchar) *mask = NULL, bool mask_out = false)
{
	Vector4f pt_camera; Vector2f pt_image;
	float depth_measure, eta, oldF, newF;
	int oldW, newW;

	// project point into image
	pt_camera = M_d * pt_model;
	if (pt_camera.z <= 0) return -1;

	pt_image.x = projParams_d.x * pt_camera.x / pt_camera.z + projParams_d.z;
	pt_image.y = projParams_d.y * pt_camera.y / pt_camera.z + projParams_d.w;
	if ((pt_image.x < 1) || (pt_image.x > imgSize.x - 2) || (pt_image.y < 1) || (pt_image.y > imgSize.y - 2)) return -1;

	// get measured depth from image
	int pt_row = (int)(pt_image.y + 0.5f);
	int pt_col = (int)(pt_image.x + 0.5f);
	depth_measure = depth[pt_col + pt_row * imgSize.x];
	//depth_measure = depth[(int)(pt_image.x + 0.5f) + (int)(pt_image.y + 0.5f) * imgSize.x];

	if (depth_measure <= 0.0f) return -1;

	// check whether voxel needs updating
	eta = depth_measure - pt_camera.z;

	/*uncomment if not work, original code*/
	if (eta < -mu) return eta;

	// compute updated SDF value and reliability
	oldF = TVoxel::valueToFloat(voxel.sdf); 
	oldW = voxel.w_depth;

	//There is no mask, we dont consider the dynamic objects
	//if we have mask, we try to detect dynamic obejcts
	if(mask == NULL){
		float th0 = 0.3;

		newF = MIN(1.0f, eta / mu);
		newW = 1;

		newF = oldW * oldF + newW * newF;
		newW = oldW + newW;
		newF /= newW;
		newW = MIN(newW, maxW);
	
		
		//write back
		voxel.sdf = TVoxel::floatToValue(newF);
		voxel.w_depth = newW;

		
	}
	else{
		newF = MIN(1.0f, eta / mu);
		newW = 1;

		bool in_mask = (mask[pt_col + pt_row * imgSize.x] == 255);

		//mask_out = true, only reconstruct static scene; else, reconstruct everything
		if(mask_out){
			if(!in_mask){
				int range = 12;
				//only check four corners, much faster
				int i, j;
				i = pt_row - range; j = pt_col - range;
				if((j > 1) && (j <= imgSize.x - 2) && (i > 1) && (i <= imgSize.y - 2))
					if(mask[j + i * imgSize.x] == 255) in_mask = true;
				i = pt_row + range; j = pt_col - range;
				if((j > 1) && (j <= imgSize.x - 2) && (i > 1) && (i <= imgSize.y - 2))
					if(mask[j + i * imgSize.x] == 255) in_mask = true;
				i = pt_row - range; j = pt_col + range;
				if((j > 1) && (j <= imgSize.x - 2) && (i > 1) && (i <= imgSize.y - 2))
					if(mask[j + i * imgSize.x] == 255) in_mask = true;
				i = pt_row + range; j = pt_col + range;
				if((j > 1) && (j <= imgSize.x - 2) && (i > 1) && (i <= imgSize.y - 2))
					if(mask[j + i * imgSize.x] == 255) in_mask = true;


				if(in_mask){
					newF = TVoxel::valueToFloat(32767);
					newW = 1;
				}
				else{
					newF = oldW * oldF + newW * newF;
					newW = oldW + newW;
					newF /= newW;
					newW = MIN(newW, maxW);				
				}
			}
			else{
				newF = TVoxel::valueToFloat(32767);
				newW = 1;				
			}
		}
		else{
			float th0 = 0.1, th1 = 2.0;
			// if((abs(newF - oldF)>th0 && abs(newF - oldF)<=th1 && !in_mask)  || abs(newF - oldF)<=th0){
			// 	//if smaller than threshod, we update normally no matter it is in mask or not, dynamic or not
			// 	newF = oldW * oldF + newW * newF;
			// 	newW = oldW + newW;
			// 	newF /= newW;
			// 	newW = MIN(newW, maxW);
			// }

			if(abs(newF - oldF)<th0 || (!in_mask && !voxel.dyna_updated_)){
				//if smaller than threshod, we update normally no matter it is in mask or not, dynamic or not
				newF = oldW * oldF + newW * newF;
				newW = oldW + newW;
				newF /= newW;
				newW = MIN(newW, maxW);
			}
			else if(abs(newF - oldF)>=th0 && !in_mask && voxel.dyna_updated_){
				//if larger than threshod, now not in mask but previsouly in mask, it means the object leaves, we reinitialize the voxel
				newF = voxel.prev_sdf_; //TVoxel::valueToFloat(32767);
				newW = voxel.prev_w_depth_;
			}
			else if(abs(newF - oldF)>=th0 && in_mask && !voxel.dyna_updated_){
				//if larger than threshod, now in mask, previously not in mask, we store the current value as a static sdf value, which may be used to recover later.
				voxel.prev_sdf_ = voxel.sdf;
				voxel.prev_w_depth_ = voxel.w_depth;
			}
		}

		voxel.dyna_updated_ = in_mask;	
	}

	//voxel default use short

	//else if(abs(newF - oldF)>=th0 && voxel.dyna_updated_) do nothing, just use new sdf
		

	//write back
	voxel.sdf = TVoxel::floatToValue(newF);
	voxel.w_depth = newW;

	return eta;
}

template<class TVoxel>
_CPU_AND_GPU_CODE_ inline float computeUpdatedVoxelDepthInfo(DEVICEPTR(TVoxel) &voxel, const THREADPTR(Vector4f) & pt_model, const CONSTPTR(Matrix4f) & M_d,
	const CONSTPTR(Vector4f) & projParams_d, float mu, int maxW, const CONSTPTR(float) *depth, const CONSTPTR(float) *confidence, const CONSTPTR(Vector2i) & imgSize, const CONSTPTR(uchar) *mask = NULL, bool mask_out = false)
{
	Vector4f pt_camera; Vector2f pt_image;
	float depth_measure, eta, oldF, newF;
	int oldW, newW, locId;

	// project point into image
	pt_camera = M_d * pt_model;
	if (pt_camera.z <= 0) return -1;

	pt_image.x = projParams_d.x * pt_camera.x / pt_camera.z + projParams_d.z;
	pt_image.y = projParams_d.y * pt_camera.y / pt_camera.z + projParams_d.w;
	if ((pt_image.x < 1) || (pt_image.x > imgSize.x - 2) || (pt_image.y < 1) || (pt_image.y > imgSize.y - 2)) return -1;

	locId = (int)(pt_image.x + 0.5f) + (int)(pt_image.y + 0.5f) * imgSize.x;
	// get measured depth from image
	depth_measure = depth[locId];
	if (depth_measure <= 0.0) return -1;

	// check whether voxel needs updating
	eta = depth_measure - pt_camera.z;
	if (eta < -mu) return eta;

	// compute updated SDF value and reliability
	oldF = TVoxel::valueToFloat(voxel.sdf); oldW = voxel.w_depth;
	newF = MIN(1.0f, eta / mu); newW = 1;

	newF = oldW * oldF + newW * newF;
	newW = oldW + newW;
	newF /= newW;
	newW = MIN(newW, maxW);

	// write back^
	voxel.sdf = TVoxel::floatToValue(newF);
	voxel.w_depth = newW;
	voxel.confidence += TVoxel::floatToValue(confidence[locId]);

	return eta;
}

template<class TVoxel>
_CPU_AND_GPU_CODE_ inline void computeUpdatedVoxelColorInfo(DEVICEPTR(TVoxel) &voxel, const THREADPTR(Vector4f) & pt_model, const CONSTPTR(Matrix4f) & M_rgb,
	const CONSTPTR(Vector4f) & projParams_rgb, float mu, uchar maxW, float eta, const CONSTPTR(Vector4u) *rgb, const CONSTPTR(Vector2i) & imgSize, bool mask_out = false)
{
	Vector4f pt_camera; Vector2f pt_image;
	Vector3f rgb_measure, oldC, newC; Vector3u buffV3u;
	float newW, oldW;

	buffV3u = voxel.clr;
	oldW = (float)voxel.w_color;

	oldC = TO_FLOAT3(buffV3u) / 255.0f;
	newC = oldC;

	pt_camera = M_rgb * pt_model;

	pt_image.x = projParams_rgb.x * pt_camera.x / pt_camera.z + projParams_rgb.z;
	pt_image.y = projParams_rgb.y * pt_camera.y / pt_camera.z + projParams_rgb.w;

	if ((pt_image.x < 1) || (pt_image.x > imgSize.x - 2) || (pt_image.y < 1) || (pt_image.y > imgSize.y - 2)) return;

	rgb_measure = TO_VECTOR3(interpolateBilinear(rgb, pt_image, imgSize)) / 255.0f;
	//rgb_measure = rgb[(int)(pt_image.x + 0.5f) + (int)(pt_image.y + 0.5f) * imgSize.x].toVector3().toFloat() / 255.0f;
	newW = 1;

	newC = oldC * oldW + rgb_measure * newW;
	newW = oldW + newW;
	newC /= newW;
	newW = MIN(newW, maxW);

	voxel.clr = TO_UCHAR3(newC * 255.0f);
	voxel.w_color = (uchar)newW;
}

template<bool hasColor, bool hasConfidence, class TVoxel> struct ComputeUpdatedVoxelInfo;


template<class TVoxel>
struct ComputeUpdatedVoxelInfo<false, false, TVoxel> {
	_CPU_AND_GPU_CODE_ static void compute(DEVICEPTR(TVoxel) & voxel, const THREADPTR(Vector4f) & pt_model,
		const CONSTPTR(Matrix4f) & M_d, const CONSTPTR(Vector4f) & projParams_d,
		const CONSTPTR(Matrix4f) & M_rgb, const CONSTPTR(Vector4f) & projParams_rgb,
		float mu, int maxW,
		const CONSTPTR(float) *depth, const CONSTPTR(float) *confidence, const CONSTPTR(Vector2i) & imgSize_d,
		const CONSTPTR(Vector4u) *rgb, const CONSTPTR(Vector2i) & imgSize_rgb, const CONSTPTR(uchar) *mask = NULL, bool mask_out = false)
	{
		computeUpdatedVoxelDepthInfo(voxel, pt_model, M_d, projParams_d, mu, maxW, depth, imgSize_d, mask, mask_out);
	}
};

//in the template, <false, false, TVoxel> means if we have color information, if we have confidence information
//normally we dont have confidence information
template<class TVoxel>
struct ComputeUpdatedVoxelInfo<true, false, TVoxel> {
	_CPU_AND_GPU_CODE_ static void compute(DEVICEPTR(TVoxel) & voxel, const THREADPTR(Vector4f) & pt_model,
		const THREADPTR(Matrix4f) & M_d, const THREADPTR(Vector4f) & projParams_d,
		const THREADPTR(Matrix4f) & M_rgb, const THREADPTR(Vector4f) & projParams_rgb,
		float mu, int maxW,
		const CONSTPTR(float) *depth, const CONSTPTR(float) *confidence, const CONSTPTR(Vector2i) & imgSize_d,
		const CONSTPTR(Vector4u) *rgb, const THREADPTR(Vector2i) & imgSize_rgb, const CONSTPTR(uchar) *mask = NULL, bool mask_out = false)
	{
		float eta = computeUpdatedVoxelDepthInfo(voxel, pt_model, M_d, projParams_d, mu, maxW, depth, imgSize_d, mask, mask_out);
		if ((eta > mu) || (fabs(eta / mu) > 0.25f)) return;
		computeUpdatedVoxelColorInfo(voxel, pt_model, M_rgb, projParams_rgb, mu, maxW, eta, rgb, imgSize_rgb, mask_out);
	}
};

template<class TVoxel>
struct ComputeUpdatedVoxelInfo<false, true, TVoxel> {
	_CPU_AND_GPU_CODE_ static void compute(DEVICEPTR(TVoxel) & voxel, const THREADPTR(Vector4f) & pt_model,
		const CONSTPTR(Matrix4f) & M_d, const CONSTPTR(Vector4f) & projParams_d,
		const CONSTPTR(Matrix4f) & M_rgb, const CONSTPTR(Vector4f) & projParams_rgb,
		float mu, int maxW,
		const CONSTPTR(float) *depth, const CONSTPTR(float) *confidence, const CONSTPTR(Vector2i) & imgSize_d,
		const CONSTPTR(Vector4u) *rgb, const CONSTPTR(Vector2i) & imgSize_rgb, const CONSTPTR(uchar) *mask = NULL, bool mask_out = false)
	{
		computeUpdatedVoxelDepthInfo(voxel, pt_model, M_d, projParams_d, mu, maxW, depth, confidence, imgSize_d, mask, mask_out);
	}
};

template<class TVoxel>
struct ComputeUpdatedVoxelInfo<true, true, TVoxel> {
	_CPU_AND_GPU_CODE_ static void compute(DEVICEPTR(TVoxel) & voxel, const THREADPTR(Vector4f) & pt_model,
		const THREADPTR(Matrix4f) & M_d, const THREADPTR(Vector4f) & projParams_d,
		const THREADPTR(Matrix4f) & M_rgb, const THREADPTR(Vector4f) & projParams_rgb,
		float mu, int maxW,
		const CONSTPTR(float) *depth, const CONSTPTR(float) *confidence, const CONSTPTR(Vector2i) & imgSize_d,
		const CONSTPTR(Vector4u) *rgb, const THREADPTR(Vector2i) & imgSize_rgb, const CONSTPTR(uchar) *mask = NULL, bool mask_out = false)
	{
		float eta = computeUpdatedVoxelDepthInfo(voxel, pt_model, M_d, projParams_d, mu, maxW, depth, confidence, imgSize_d, mask, mask_out);
		if ((eta > mu) || (fabs(eta / mu) > 0.25f)) return;
		computeUpdatedVoxelColorInfo(voxel, pt_model, M_rgb, projParams_rgb, mu, maxW, eta, rgb, imgSize_rgb, mask_out);
	}
};

_CPU_AND_GPU_CODE_ inline void buildHashAllocAndVisibleTypePP(DEVICEPTR(uchar) *entriesAllocType, DEVICEPTR(uchar) *entriesVisibleType, int x, int y,
	DEVICEPTR(Vector4s) *blockCoords, const CONSTPTR(float) *depth, Matrix4f invM_d, Vector4f projParams_d, float mu, Vector2i imgSize,
	float oneOverVoxelSize, const CONSTPTR(ITMHashEntry) *hashTable, float viewFrustum_min, float viewFrustum_max)
{
	float depth_measure; unsigned int hashIdx; int noSteps;
	Vector4f pt_camera_f; Vector3f point_e, point, direction; Vector3s blockPos;

	depth_measure = depth[x + y * imgSize.x];
	if (depth_measure <= 0 || (depth_measure - mu) < 0 || (depth_measure - mu) < viewFrustum_min || (depth_measure + mu) > viewFrustum_max) return;

	pt_camera_f.z = depth_measure;
	pt_camera_f.x = pt_camera_f.z * ((float(x) - projParams_d.z) * projParams_d.x);
	pt_camera_f.y = pt_camera_f.z * ((float(y) - projParams_d.w) * projParams_d.y);

	float norm = sqrt(pt_camera_f.x * pt_camera_f.x + pt_camera_f.y * pt_camera_f.y + pt_camera_f.z * pt_camera_f.z);

	Vector4f pt_buff;
	
	pt_buff = pt_camera_f * (1.0f - mu / norm); pt_buff.w = 1.0f;
	point = TO_VECTOR3(invM_d * pt_buff) * oneOverVoxelSize;

	pt_buff = pt_camera_f * (1.0f + mu / norm); pt_buff.w = 1.0f;
	point_e = TO_VECTOR3(invM_d * pt_buff) * oneOverVoxelSize;

	direction = point_e - point;
	norm = sqrt(direction.x * direction.x + direction.y * direction.y + direction.z * direction.z);
	noSteps = (int)ceil(2.0f*norm);

	direction /= (float)(noSteps - 1);

	//add neighbouring blocks
	for (int i = 0; i < noSteps; i++)
	{
		blockPos = TO_SHORT_FLOOR3(point);

		//compute index in hash table
		hashIdx = hashIndex(blockPos);

		//check if hash table contains entry
		bool isFound = false;

		ITMHashEntry hashEntry = hashTable[hashIdx];

		if (IS_EQUAL3(hashEntry.pos, blockPos) && hashEntry.ptr >= -1)
		{
			//entry has been streamed out but is visible or in memory and visible
			entriesVisibleType[hashIdx] = (hashEntry.ptr == -1) ? 2 : 1;

			isFound = true;
		}

		if (!isFound)
		{
			bool isExcess = false;
			if (hashEntry.ptr >= -1) //seach excess list only if there is no room in ordered part
			{
				while (hashEntry.offset >= 1)
				{
					hashIdx = SDF_BUCKET_NUM + hashEntry.offset - 1;
					hashEntry = hashTable[hashIdx];

					if (IS_EQUAL3(hashEntry.pos, blockPos) && hashEntry.ptr >= -1)
					{
						//entry has been streamed out but is visible or in memory and visible
						entriesVisibleType[hashIdx] = (hashEntry.ptr == -1) ? 2 : 1;

						isFound = true;
						break;
					}
				}

				isExcess = true;
			}

			if (!isFound) //still not found
			{
				entriesAllocType[hashIdx] = isExcess ? 2 : 1; //needs allocation 
				if (!isExcess) entriesVisibleType[hashIdx] = 1; //new entry is visible

				blockCoords[hashIdx] = Vector4s(blockPos.x, blockPos.y, blockPos.z, 1);
			}
		}

		point += direction;
	}
}

template<bool useSwapping>
_CPU_AND_GPU_CODE_ inline void checkPointVisibility(THREADPTR(bool) &isVisible, THREADPTR(bool) &isVisibleEnlarged,
	const THREADPTR(Vector4f) &pt_image, const CONSTPTR(Matrix4f) & M_d, const CONSTPTR(Vector4f) &projParams_d,
	const CONSTPTR(Vector2i) &imgSize)
{
	Vector4f pt_buff;

	pt_buff = M_d * pt_image;

	if (pt_buff.z < 1e-10f) return;

	pt_buff.x = projParams_d.x * pt_buff.x / pt_buff.z + projParams_d.z;
	pt_buff.y = projParams_d.y * pt_buff.y / pt_buff.z + projParams_d.w;

	if (pt_buff.x >= 0 && pt_buff.x < imgSize.x && pt_buff.y >= 0 && pt_buff.y < imgSize.y) { isVisible = true; isVisibleEnlarged = true; }
	else if (useSwapping)
	{
		Vector4i lims;
		lims.x = -imgSize.x / 8; lims.y = imgSize.x + imgSize.x / 8;
		lims.z = -imgSize.y / 8; lims.w = imgSize.y + imgSize.y / 8;

		if (pt_buff.x >= lims.x && pt_buff.x < lims.y && pt_buff.y >= lims.z && pt_buff.y < lims.w) isVisibleEnlarged = true;
	}
}

template<bool useSwapping>
_CPU_AND_GPU_CODE_ inline void checkBlockVisibility(THREADPTR(bool) &isVisible, THREADPTR(bool) &isVisibleEnlarged,
	const THREADPTR(Vector3s) &hashPos, const CONSTPTR(Matrix4f) & M_d, const CONSTPTR(Vector4f) &projParams_d,
	const CONSTPTR(float) &voxelSize, const CONSTPTR(Vector2i) &imgSize)
{
	Vector4f pt_image;
	float factor = (float)SDF_BLOCK_SIZE * voxelSize;

	isVisible = false; isVisibleEnlarged = false;

	// 0 0 0
	pt_image.x = (float)hashPos.x * factor; pt_image.y = (float)hashPos.y * factor;
	pt_image.z = (float)hashPos.z * factor; pt_image.w = 1.0f;
	checkPointVisibility<useSwapping>(isVisible, isVisibleEnlarged, pt_image, M_d, projParams_d, imgSize);
	if (isVisible) return;

	// 0 0 1
	pt_image.z += factor;
	checkPointVisibility<useSwapping>(isVisible, isVisibleEnlarged, pt_image, M_d, projParams_d, imgSize);
	if (isVisible) return;

	// 0 1 1
	pt_image.y += factor;
	checkPointVisibility<useSwapping>(isVisible, isVisibleEnlarged, pt_image, M_d, projParams_d, imgSize);
	if (isVisible) return;

	// 1 1 1
	pt_image.x += factor;
	checkPointVisibility<useSwapping>(isVisible, isVisibleEnlarged, pt_image, M_d, projParams_d, imgSize);
	if (isVisible) return;

	// 1 1 0 
	pt_image.z -= factor;
	checkPointVisibility<useSwapping>(isVisible, isVisibleEnlarged, pt_image, M_d, projParams_d, imgSize);
	if (isVisible) return;

	// 1 0 0 
	pt_image.y -= factor;
	checkPointVisibility<useSwapping>(isVisible, isVisibleEnlarged, pt_image, M_d, projParams_d, imgSize);
	if (isVisible) return;

	// 0 1 0
	pt_image.x -= factor; pt_image.y += factor;
	checkPointVisibility<useSwapping>(isVisible, isVisibleEnlarged, pt_image, M_d, projParams_d, imgSize);
	if (isVisible) return;

	// 1 0 1
	pt_image.x += factor; pt_image.y -= factor; pt_image.z += factor;
	checkPointVisibility<useSwapping>(isVisible, isVisibleEnlarged, pt_image, M_d, projParams_d, imgSize);
	if (isVisible) return;
}
