// Copyright (C) 2002-2012 Nikolaus Gebhardt
// This file is part of the "Irrlicht Engine".
// For conditions of distribution and use, see copyright notice in irrlicht.h

#ifndef __C_ANIMATED_MESH_SCENE_NODE_H_INCLUDED__
#define __C_ANIMATED_MESH_SCENE_NODE_H_INCLUDED__

#include "IAnimatedMeshSceneNode.h"
#include "IAnimatedMesh.h"

#include "matrix4.h"


namespace irr
{
namespace scene
{
	class IDummyTransformationSceneNode;

	class CAnimatedMeshSceneNode : public IAnimatedMeshSceneNode
	{
	public:

		//! constructor
		CAnimatedMeshSceneNode(IAnimatedMesh* mesh, ISceneNode* parent, ISceneManager* mgr,	s32 id,
			const core::vector3df& position = core::vector3df(0,0,0),
			const core::vector3df& rotation = core::vector3df(0,0,0),
			const core::vector3df& scale = core::vector3df(1.0f, 1.0f, 1.0f));

		//! destructor
		virtual ~CAnimatedMeshSceneNode();

		//! sets the current frame. from now on the animation is played from this frame.
		void setCurrentFrame(f32 frame) override;

		//! frame
		void OnRegisterSceneNode() override;

		//! OnAnimate() is called just before rendering the whole scene.
		void OnAnimate(u32 timeMs) override;

		//! renders the node.
		void render() override;

		//! returns the axis aligned bounding box of this node
		const core::aabbox3d<f32>& getBoundingBox() const override;

		//! sets the frames between the animation is looped.
		//! the default is 0 - MaximalFrameCount of the mesh.
		//! NOTE: setMesh will also change this value and set it to the full range of animations of the mesh
		bool setFrameLoop(s32 begin, s32 end) override;

		//! Sets looping mode which is on by default. If set to false,
		//! animations will not be looped.
		void setLoopMode(bool playAnimationLooped) override;

		//! returns the current loop mode
		bool getLoopMode() const override;

		//! Sets a callback interface which will be called if an animation
		//! playback has ended. Set this to 0 to disable the callback again.
		void setAnimationEndCallback(IAnimationEndCallBack* callback=0) override;

		//! sets the speed with which the animation is played
		//! NOTE: setMesh will also change this value and set it to the default speed of the mesh
		void setAnimationSpeed(f32 framesPerSecond) override;

		//! gets the speed with which the animation is played
		f32 getAnimationSpeed() const override;

		//! returns the material based on the zero based index i. To get the amount
		//! of materials used by this scene node, use getMaterialCount().
		//! This function is needed for inserting the node into the scene hierarchy on a
		//! optimal position for minimizing renderstate changes, but can also be used
		//! to directly modify the material of a scene node.
		video::SMaterial& getMaterial(u32 i) override;

		//! returns amount of materials used by this scene node.
		u32 getMaterialCount() const override;

		//! Returns a pointer to a child node, which has the same transformation as
		//! the corresponding joint, if the mesh in this scene node is a skinned mesh.
		IBoneSceneNode* getJointNode(const c8* jointName) override;

		//! same as getJointNode(const c8* jointName), but based on id
		IBoneSceneNode* getJointNode(u32 jointID) override;

		//! Gets joint count.
		u32 getJointCount() const override;

		//! Removes a child from this scene node.
		//! Implemented here, to be able to remove the shadow properly, if there is one,
		//! or to remove attached child.
		bool removeChild(ISceneNode* child) override;

		//! Returns the current displayed frame number.
		f32 getFrameNr() const override;
		//! Returns the current start frame number.
		s32 getStartFrame() const override;
		//! Returns the current end frame number.
		s32 getEndFrame() const override;

		//! Sets if the scene node should not copy the materials of the mesh but use them in a read only style.
		/* In this way it is possible to change the materials a mesh causing all mesh scene nodes
		referencing this mesh to change too. */
		void setReadOnlyMaterials(bool readonly) override;

		//! Returns if the scene node should not copy the materials of the mesh but use them in a read only style
		bool isReadOnlyMaterials() const override;

		//! Sets a new mesh
		void setMesh(IAnimatedMesh* mesh) override;

		//! Returns the current mesh
		IAnimatedMesh* getMesh(void) override { return Mesh; }

		//! Returns type of the scene node
		ESCENE_NODE_TYPE getType() const override { return ESNT_ANIMATED_MESH; }

		//! updates the absolute position based on the relative and the parents position
		void updateAbsolutePosition() override;


		//! Set the joint update mode (0-unused, 1-get joints only, 2-set joints only, 3-move and set)
		void setJointMode(E_JOINT_UPDATE_ON_RENDER mode) override;

		//! Sets the transition time in seconds (note: This needs to enable joints, and setJointmode maybe set to 2)
		//! you must call animateJoints(), or the mesh will not animate
		void setTransitionTime(f32 Time) override;

		//! updates the joint positions of this mesh
		void animateJoints(bool CalculateAbsolutePositions=true) override;

		//! render mesh ignoring its transformation. Used with ragdolls. (culling is unaffected)
		void setRenderFromIdentity( bool On ) override;

		//! Creates a clone of this scene node and its children.
		/** \param newParent An optional new parent.
		\param newManager An optional new scene manager.
		\return The newly created clone of this node. */
		ISceneNode* clone(ISceneNode* newParent=0, ISceneManager* newManager=0) override;

	private:

		//! Get a static mesh for the current frame of this animated mesh
		IMesh* getMeshForCurrentFrame();

		void buildFrameNr(u32 timeMs);
		void checkJoints();
		void beginTransition();

		core::array<video::SMaterial> Materials;
		core::aabbox3d<f32> Box;
		IAnimatedMesh* Mesh;

		s32 StartFrame;
		s32 EndFrame;
		f32 FramesPerSecond;
		f32 CurrentFrameNr;

		u32 LastTimeMs;
		u32 TransitionTime; //Transition time in millisecs
		f32 Transiting; //is mesh transiting (plus cache of TransitionTime)
		f32 TransitingBlend; //0-1, calculated on buildFrameNr

		//0-unused, 1-get joints only, 2-set joints only, 3-move and set
		E_JOINT_UPDATE_ON_RENDER JointMode;
		bool JointsUsed;

		bool Looping;
		bool ReadOnlyMaterials;
		bool RenderFromIdentity;

		IAnimationEndCallBack* LoopCallBack;
		s32 PassCount;

		core::array<IBoneSceneNode* > JointChildSceneNodes;
		core::array<core::matrix4> PretransitingSave;
	};

} // end namespace scene
} // end namespace irr

#endif

