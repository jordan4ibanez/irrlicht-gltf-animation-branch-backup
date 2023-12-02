// Copyright (C) 2002-2012 Nikolaus Gebhardt
// This file is part of the "Irrlicht Engine".
// For conditions of distribution and use, see copyright notice in irrlicht.h

#ifndef __C_MESH_MANIPULATOR_H_INCLUDED__
#define __C_MESH_MANIPULATOR_H_INCLUDED__

#include "IMeshManipulator.h"

namespace irr
{
namespace scene
{

//! An interface for easy manipulation of meshes.
/** Scale, set alpha value, flip surfaces, and so on. This exists for fixing
problems with wrong imported or exported meshes quickly after loading. It is
not intended for doing mesh modifications and/or animations during runtime.
*/
class CMeshManipulator : public IMeshManipulator
{
public:
	//! Recalculates all normals of the mesh.
	/** \param mesh: Mesh on which the operation is performed.
	\param smooth: Whether to use smoothed normals. */
	void recalculateNormals(scene::IMesh* mesh, bool smooth = false, bool angleWeighted = false) const override;

	//! Recalculates all normals of the mesh buffer.
	/** \param buffer: Mesh buffer on which the operation is performed.
	\param smooth: Whether to use smoothed normals. */
	void recalculateNormals(IMeshBuffer* buffer, bool smooth = false, bool angleWeighted = false) const override;

	//! Clones a static IMesh into a modifiable SMesh.
	SMesh* createMeshCopy(scene::IMesh* mesh) const override;

	//! Returns amount of polygons in mesh.
	s32 getPolyCount(scene::IMesh* mesh) const override;

	//! Returns amount of polygons in mesh.
	s32 getPolyCount(scene::IAnimatedMesh* mesh) const override;

	//! create a new AnimatedMesh and adds the mesh to it
	IAnimatedMesh * createAnimatedMesh(scene::IMesh* mesh,scene::E_ANIMATED_MESH_TYPE type) const override;
};

} // end namespace scene
} // end namespace irr


#endif
