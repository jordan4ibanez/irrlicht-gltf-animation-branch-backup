#include "CGLTFMeshFileLoader.h"
#include "CMeshBuffer.h"
#include "coreutil.h"
#include "CSkinnedMesh.h"
#include "IReadFile.h"
#include "irrTypes.h"
#include "path.h"
#include "S3DVertex.h"
#include "SAnimatedMesh.h"
#include "SColor.h"
#include "SMesh.h"
#include "SSkinMeshBuffer.h"
#include "vector3d.h"
#include "quaternion.h"

#define TINYGLTF_IMPLEMENTATION
#include <tiny_gltf.h>

#include <cstddef>
#include <cstring>
#include <memory>
#include <set>
#include <string>
#include <tuple>
#include <vector>
#include <iostream>

// No idea if this is necessary, maybe remove this?
#define _USE_MATH_DEFINES
#include <math.h>
const float DEG = 180.0 / M_PI;

template <class T>
struct Span
{
	T* buffer = nullptr;
	std::size_t size = 0;
};

void printVec(irr::core::vector3df input, std::string name = "Vector") {
    std::cout << name <<"(" << input.X << ", " << input.Y << ", " << input.Z << ")\n";
}
void printQuat(irr::core::quaternion input, std::string name = "QUATERNION") {
    std::cout << name << " (";
    std::cout << input.X << ", " << input.Y << ", " << input.Z << ", " << input.W << ")\n";
}
void printMatrix4(irr::core::matrix4 i, std::string name = "MATRIX")
{
    std::cout << name << ":\n" <<
    i[0] << ", " << i[1]  << ", " << i[2]  << ", " << i[3]  << "|\n" <<
    i[4] << ", " << i[5]  << ", " << i[6]  << ", " << i[7]  << "|\n" <<
    i[8] << ", " << i[9]  << ", " << i[10] << ", " << i[11] << "|\n" <<
    i[12] << ", "<< i[13] << ", " << i[14] << ", " << i[15] << "|\n";
    
}


class BufferOffset
{
public:
	BufferOffset(const std::vector<unsigned char>& buf,
			const std::size_t offset)
		: m_buf(buf)
		, m_offset(offset)
	{
	}

	BufferOffset(const BufferOffset& other, const std::size_t fromOffset)
		: m_buf(other.m_buf)
		, m_offset(other.m_offset + fromOffset)
	{
	}

	unsigned char at(const std::size_t fromOffset) const
	{
		return m_buf.at(m_offset + fromOffset);
	}

private:
	const std::vector<unsigned char>& m_buf;
	std::size_t m_offset;
};

int getByteOffset(tinygltf::Accessor accessor, tinygltf::BufferView bufferView) {
    const int accessorByteOffset = accessor.byteOffset;
    const int viewOffset = bufferView.byteOffset;

    // std::cout <<  "getByteOffsetDebug: " << accessorByteOffset << " | " << viewOffset << "\n";

    return accessorByteOffset + viewOffset;
};

//! This is written out extremely verbosely for maintainability, DO NOT shorten this
int getByteStride(tinygltf::Accessor accessor, tinygltf::BufferView bufferView) {
    int componentMembers = tinygltf::GetNumComponentsInType(accessor.type);
    int componentSize = tinygltf::GetComponentSizeInBytes(accessor.componentType);
    int byteStride = bufferView.byteStride;
    int returningValue = 0;

    assert(byteStride % componentSize == 0);

    // Undefined - tightly packed
    if (byteStride == 0) {
        returningValue = componentSize * componentMembers;
    } 
    // Defined - interleaved
    else {
        returningValue = byteStride;
        //! THIS IS REQUIRED, in gltf spec this MUST be a multiple of it's component type!
        assert(returningValue % componentSize == 0);
    }
    assert(returningValue != 0);
    return returningValue;
};


auto validateValueFromAccessor = [](tinygltf::Accessor accessor, double value) {
    if (accessor.minValues.size() > 0) {
        // std::cout << "MIN: " << accessor.minValues.at(0) << "\n";
        assert(value >= accessor.minValues.at(0) - 0.000001);
    }
    if (accessor.maxValues.size() > 0) {
        // std::cout << "MAX: " << accessor.maxValues.at(0) << "\n";
        assert(value <= accessor.maxValues.at(0) + 0.000001);
    }
};

// A helper function to disable tinygltf embedded image loading
bool dummyImageLoader(tinygltf::Image *a, const int b, std::string *c,
	std::string *d, int e, int f, const unsigned char * g,
	int h, void *user_pointer)
{
	return true;
};


template <class T>
static T rawReadPrimitive(const BufferOffset& readFrom)
{
	unsigned char d[sizeof(T)]{};
	for (std::size_t i = 0; i < sizeof(T); ++i) {
		d[i] = readFrom.at(i);
	}
	T dest;
	std::memcpy(&dest, d, sizeof(dest));
	return dest;
}

// These values become promoted or demoted into whatever def type they are in
// These values corrispond with tinygltf::TINYGLTF_COMPONENT_TYPE
double readPrimitive(tinygltf::Accessor accessor, const BufferOffset& readFrom) {
    switch(accessor.componentType) {
        case (5120): {
            return rawReadPrimitive<char>(readFrom);
        }
        case (5121): {
            return rawReadPrimitive<u_char>(readFrom);
        }
        case (5122): {
            return rawReadPrimitive<short>(readFrom);
        }
        case (5123): {
            return rawReadPrimitive<u_short>(readFrom);
        }
        case (5124): {
            return rawReadPrimitive<int>(readFrom);
        }
        case (5125): {
            return rawReadPrimitive<u_int>(readFrom);
        }
        case (5126): {
            return rawReadPrimitive<float>(readFrom);
        }
        case (5130): {
            return rawReadPrimitive<double>(readFrom);
        }
        default: return 0;
    }
}



namespace irr
{

//! This is translation of Auri's GLM function into irrlicht
core::matrix4 getOffsetMatrix(
    const core::vector3df position,
    const core::quaternion rotation = core::quaternion(core::vector3df(0)),
    const core::vector3df scale = core::vector3df(1)){
        // Diagonal matrix 1s - glm::mat4(1.0f) equivelant
        const auto baseMat4 = core::matrix4()
            .makeIdentity()
            .setTranslation(position);
        // Convert the quaternion to a 4d matrix
        const auto rotationMatrix = rotation.getMatrix();

        return (baseMat4 * rotationMatrix).setScale(scale);
};


namespace scene
{



static bool tryParseGLTF(io::IReadFile* file, tinygltf::Model& model)
{
	tinygltf::TinyGLTF loader {};

	// Stop embedded textures from making model fail to load
	loader.SetImageLoader(dummyImageLoader, nullptr);

	std::string err {};
	std::string warn {};

	auto buf = std::make_unique<char[]>(file->getSize());
	file->read(buf.get(), file->getSize());

	if (err != "" || warn != "") {
		return false;
	}

	return loader.LoadASCIIFromString(&model, &err, &warn, buf.get(),
		file->getSize(), "", 1);
}

static core::vector2df readVec2DF(const BufferOffset& readFrom)
{
	return core::vector2df(rawReadPrimitive<float>(readFrom),
		rawReadPrimitive<float>(BufferOffset( readFrom, sizeof(float))));

}
// TODO: Scale inversion needs to be tested with normals!, turn on the fancy shaders
static core::vector3df readVec3DF(const BufferOffset& readFrom,
		core::vector3df scale = core::vector3df(1))
{
    // glTF's coordinate system is right-handed, Irrlicht's is left-handed
	// glTF's +Z axis corresponds to Irrlicht's -Z axis
    // scale.X *= -1;
    // scale.Z *= -1;

	return core::vector3df(
		rawReadPrimitive<float>(readFrom),
		rawReadPrimitive<float>(BufferOffset(readFrom, sizeof(float))),
		rawReadPrimitive<float>(BufferOffset(readFrom, 2 *
		sizeof(float)))) * scale;
}


static core::quaternion readQuaternion(const BufferOffset& readFrom, core::vector3df scale = core::vector3df(1))
{
    // glTF's coordinate system is right-handed, Irrlicht's is left-handed
	// glTF's +Z axis corresponds to Irrlicht's -Z axis
    //scale.Y *= -1;
	return core::quaternion(
		rawReadPrimitive<float>(readFrom),
		rawReadPrimitive<float>(BufferOffset(readFrom, sizeof(float))),
		rawReadPrimitive<float>(BufferOffset(readFrom, 2 * sizeof(float))),
        rawReadPrimitive<float>(BufferOffset(readFrom, 3 * sizeof(float))));
}

static core::quaternion readBoneDataQuaternion(tinygltf::Accessor accessor, const BufferOffset& readFrom)
{
    // We need to do this the WRONG way, we need the ACTUAL byte length
    const int sizeInBytes = tinygltf::GetComponentSizeInBytes(accessor.componentType);

    return core::quaternion(
    readPrimitive(accessor, readFrom),
    readPrimitive(accessor, BufferOffset(readFrom, sizeInBytes)),
    readPrimitive(accessor, BufferOffset(readFrom, 2 * sizeInBytes)),
    readPrimitive(accessor, BufferOffset(readFrom, 3 * sizeInBytes))
    );
}



static core::matrix4 readMatrix4(const BufferOffset& readFrom)
{
    // S stands for storage, you can probably see why I made it one letter
    float s[16];
    for (int i = 0; i < 16; i++) {
        s[i] = rawReadPrimitive<float>(BufferOffset(readFrom, i * sizeof(float)));
    }
    return core::matrix4(
        s[0], s[1], s[2], s[3],
        s[4], s[5], s[6], s[7],
        s[8], s[9], s[10],s[11],
        s[12],s[13],s[14],s[15]
    );
}

static void recurseBoneTransforms(
    std::map<int, int> &tracker,
    std::map<int, int> irrlichtToGltf,
    std::map<int, int> gltfToIrrlicht,
    const tinygltf::Model model,
    const scene::CSkinnedMesh *animatedMesh,
    const tinygltf::Buffer ibmBuffer,
    const int ibmByteOffset,
    const int ibmByteStride,
    const ushort bone,
    const core::matrix4 parentWorldTransform,
    const core::matrix4 globalInverseWorldTransform
    ) {

    bool goodToGo = false;
    
    if (tracker[bone] == 0) {
        tracker[bone] = -1;
    } else {
        // std::cout << "ALREADY INDEXED JOINT " << bone << "\n";
        return;
    }
    

    // std::cout << "RECURSING BONE " << bone << "\n";
    

    auto thisJoint = animatedMesh->getAllJoints()[gltfToIrrlicht[bone]];

    //! Extreme debugging wooooooo
    if (thisJoint->Name != "Body") {
        // return;
    }

    const auto thisNode = model.nodes[bone];

    core::vector3df thisTranslation = thisNode.translation.size() == 0 ? core::vector3df(0) : 
        core::vector3df(thisNode.translation.at(0),thisNode.translation.at(1),thisNode.translation.at(2));

    core::quaternion thisRotation = thisNode.rotation.size() == 0 ? core::quaternion(core::vector3df(0)) : 
        core::quaternion(thisNode.rotation.at(0),thisNode.rotation.at(1),thisNode.rotation.at(2),thisNode.rotation.at(3));

    core::vector3df thisScale = thisNode.scale.size() == 0 ? core::vector3df(1) :
        core::vector3df(thisNode.scale.at(0),thisNode.scale.at(1),thisNode.scale.at(2));

    std::cout << thisJoint->Name.c_str() << "\n";


    // Undo this in the future, should take in the order automagically when doing the initial bone parent hierarchy thing
    int thisBoneMultiplier = 0;
    bool FAILED_TO_FIND_INDEX = true;
    for (int i = 0; i < model.skins[0].joints.size(); i++) {
        if (bone == model.skins[0].joints[i]) {
            thisBoneMultiplier = i;
            FAILED_TO_FIND_INDEX = false;
        }
    }

    // Sometimes models just have random pointer data shoveled in so let's not parse that and move on
    if (FAILED_TO_FIND_INDEX) {
        std::cout << "FAILED TO PARSE DAT BONE\n";
        return;
    }
    
    thisRotation.Z *= -1.0;
    // thisRotation.X *= -1.0;
    thisTranslation.Z *= -1.0;
    // thisTranslation.X *= -1.0;

    core::matrix4 positionMatrix;
    positionMatrix.setTranslation( thisTranslation );
    core::matrix4 scaleMatrix;
    scaleMatrix.setScale( thisScale );
    core::matrix4 rotationMatrix;
    thisRotation.getMatrix_transposed(rotationMatrix);
    
    core::matrix4 localMatrix = positionMatrix * scaleMatrix * rotationMatrix;

    thisJoint->Animatedposition = thisTranslation;
    thisJoint->Animatedrotation = thisRotation;
    thisJoint->Animatedscale = thisScale;

    thisJoint->LocalMatrix = localMatrix;
    thisJoint->GlobalMatrix = thisJoint->LocalMatrix * parentWorldTransform;
    

    const auto worldTransform = thisJoint->GlobalMatrix;



    //! This is debugging print
    const bool DEBUG_JOINT_HEIRARCHY = false;
    if (DEBUG_JOINT_HEIRARCHY){
    std::cout << "BONE " << thisNode.name << " HAS CHILDREN: ( ";
    const int lastValue = thisNode.children.size() - 1;
    int count = 0;
    for (int thisChild : thisNode.children) {

        std::cout << model.nodes[thisChild].name;

        if (count != lastValue) {
            std::cout << ",";
        }

        count++;
    }
    std::cout << " )\n";
    }
    //! End debugging print

    for (const int thisChild : thisNode.children) {

        recurseBoneTransforms(
            tracker,
            irrlichtToGltf,
            gltfToIrrlicht,
            model,
            animatedMesh,
            ibmBuffer,
            ibmByteOffset,
            ibmByteStride,
            thisChild,
            worldTransform,
            globalInverseWorldTransform
        );
    }   
};

static core::vector3df getScale(const tinygltf::Model& model, const std::size_t meshIndex)
{
	if (model.nodes[meshIndex].scale.size() == 3) {
        const auto gottenScale = model.nodes[meshIndex].scale;
        return core::vector3df(
            gottenScale.at(0),
            gottenScale.at(1),
            gottenScale.at(2)
        );
	}
	return core::vector3df(1);
}

core::vector3df getTranslation(const tinygltf::Model& model, std::size_t meshIndex) {

    if (model.nodes[meshIndex].translation.size() == 3) {
        return core::vector3df(
            model.nodes[meshIndex].translation.at(0),
            model.nodes[meshIndex].translation.at(1),
            model.nodes[meshIndex].translation.at(2)
        );
    }
    return core::vector3df(0,0,0);
}

core::vector3df getRotation(const tinygltf::Model& model, std::size_t meshIndex) {

    if (model.nodes[meshIndex].rotation.size() == 4) {
        auto test = model.nodes[meshIndex].rotation;
        auto result = core::vector3df(0);
        core::quaternion(test.at(0), test.at(1), test.at(2), test.at(3)).toEuler(result);
        //result.operator*=(core::vector3df(-DEG,DEG,DEG));
        return result;
    }
    return core::vector3df(0,0,0);
}

static void copyPositions(const tinygltf::Model& model,
	std::vector<irr::video::S3DVertex>& vertexBufferVector, 
    const std::size_t accessorId, const std::size_t meshIndex)
{

    const auto& accessor = model.accessors[accessorId];
	const auto& view = model.bufferViews[accessor.bufferView];
	const auto& buffer = model.buffers[view.buffer];
	const auto count = model.accessors[accessorId].count;
    const int byteStride = getByteStride(accessor, view);
    const int byteOffset = getByteOffset(accessor, view);
    
    const int inverseCount = count - 1;

	for (std::size_t i = 0; i < count; i++) {

		vertexBufferVector.at(i).Pos = readVec3DF(BufferOffset( buffer.data, byteOffset + (byteStride * i)));

        // Bolt on test
        // vertexBufferVector.at(i).Pos.X *= -1.0;
        vertexBufferVector.at(i).Pos.Z *= -1.0;

	}
}

static void copyNormals(const tinygltf::Model& model,
std::vector<irr::video::S3DVertex>& vertexBufferVector,
const std::size_t accessorId)
{
    const auto& accessor = model.accessors[accessorId];
	const auto& view = model.bufferViews[accessor.bufferView];
	const auto& buffer = model.buffers[view.buffer];
	const auto count = model.accessors[accessorId].count;
    const int byteStride = getByteStride(accessor, view);
    const int byteOffset = getByteOffset(accessor, view);

    // std::cout << "stride is : " << stride << " size of float: " << sizeof(float) << "\n";
	
	for (std::size_t i = 0; i < count; i++) {
		vertexBufferVector.at(i).Normal = readVec3DF(BufferOffset( buffer.data, byteOffset + (byteStride * i)));
	}
}

static void copyTCoords(const tinygltf::Model& model,
	std::vector<irr::video::S3DVertex>& vertexBufferVector,
    const std::size_t accessorId)
{
    const auto& accessor = model.accessors[accessorId];
	const auto& view = model.bufferViews[accessor.bufferView];
	const auto& buffer = model.buffers[view.buffer];
	const auto count = model.accessors[accessorId].count;
    const int byteStride = getByteStride(accessor, view);
    const int byteOffset = getByteOffset(accessor, view);

    // std::cout << "stride is: " << stride << "\n";

	for (std::size_t i = 0; i < count; ++i) {
		vertexBufferVector.at(i).TCoords = readVec2DF(BufferOffset(buffer.data, byteOffset + (byteStride * i)));
	}
}

static void getIndices(const tinygltf::Model& model,
	const std::size_t accessorId, core::array<u16>& indicesBuffer)
{
    const auto& accessor = model.accessors[accessorId];
	const auto& view = model.bufferViews[accessor.bufferView];
	const auto& buffer = model.buffers[view.buffer];
	const auto count = model.accessors[accessorId].count;

    const int byteStride = getByteStride(accessor, view);
    const int byteOffset = getByteOffset(accessor, view);

    // std::cout << "indice stride is: " << stride << "\n";

    // Preallocate to the vector
    indicesBuffer.set_used(count);

    // Now grab the vector pointer
    auto indicesBufferPointer = indicesBuffer.pointer();

    // Use pointer arithmatic to allocate
    const int inverseCount = count - 1;
    for (std::size_t i = 0; i < count; i++) {
        indicesBufferPointer[inverseCount - i] = readPrimitive(accessor, BufferOffset(buffer.data, byteOffset + (byteStride * i)));
    }
}

//Returns a tuple of the current counts (current_vertex_index,
//	current_normals_index, current_tcoords_index)
static void getVertices
( const tinygltf::Model& model, const std::size_t accessorId,
	core::array<irr::video::S3DVertex>& verticesBuffer, 
	std::size_t meshIndex, std::size_t primitiveIndex)
{
    
    // We must grab the initial count out of the buffer to allocate the vector of objects
    const auto& positionsBufferView = model.bufferViews[model.accessors[accessorId].bufferView];
	const auto& positionsBuffer = model.buffers[positionsBufferView.buffer];
	const auto positionsCount = model.accessors[accessorId].count;

    // Preallocate a vector of irr::video::S3DVertex objects
    std::vector<irr::video::S3DVertex> vertexBufferVector = std::vector<irr::video::S3DVertex>(positionsCount, irr::video::S3DVertex());

    // TRS - Transformation Rotation Scale
	copyPositions(model, vertexBufferVector, accessorId, meshIndex);

    const auto tCoordsField = model.meshes[meshIndex].primitives[primitiveIndex].attributes.find("TEXCOORD_0");
	if (tCoordsField != model.meshes[meshIndex].primitives[primitiveIndex].attributes.end()) {
		copyTCoords(model, vertexBufferVector, tCoordsField->second);
	}
    
	const auto normalsField = model.meshes[meshIndex].primitives[primitiveIndex].attributes.find("NORMAL");
	if (normalsField != model.meshes[meshIndex].primitives[primitiveIndex].attributes.end()) {
		copyNormals(model, vertexBufferVector, normalsField->second);
	}

    // Now copy all the vertex data into the main buffer
    for (int i = 0; i < positionsCount; i++){
        verticesBuffer.push_back(vertexBufferVector.at(i));
    }

}

CGLTFMeshFileLoader::CGLTFMeshFileLoader()
{
}

bool CGLTFMeshFileLoader::isALoadableFileExtension(
	const io::path& filename) const
{
	return core::hasFileExtension(filename, "gltf");
}


//* This is recursing the tree of parent-children indexes in gltf
void recurseParentChildrenHierarchyInsertion(
    scene::CSkinnedMesh* animatedMesh,
    const tinygltf::Model model,
    const tinygltf::Skin thisSkin,
    const int thisBoneGltfIndex,
    CSkinnedMesh::SJoint* parentIrrlichtBone,
    std::map<int, int> &boneInsertionTracker,
    std::map<int, int> &irrlichtToGltf,
    std::map<int, int> &gltfToIrrlicht) {


    if (boneInsertionTracker[thisBoneGltfIndex] == 0) {
        boneInsertionTracker[thisBoneGltfIndex] = -1;
    } else {
        // std::cout << "ALREADY ADDED JOINT " << thisBoneGltfIndex << "\n";
        return;
    }

    // Access the bone node from the model
    const tinygltf::Node gltfBoneNode = model.nodes[thisBoneGltfIndex];

    // Create the child from the root node
    const auto thisJoint = animatedMesh->addJoint(parentIrrlichtBone);
    thisJoint->Name = gltfBoneNode.name.c_str();

    // We want to get the current index that's added in, this is extremely important
    const int irrlichtJointIndex = animatedMesh->getAllJoints().size() - 1;

    // std::cout << "gltf: " << thisBoneGltfIndex << " is irr " << irrlichtJointIndex << "\n";

    // Will keep the index of this irrlicht bone aligned to gltf
    irrlichtToGltf[irrlichtJointIndex] = thisBoneGltfIndex;
    // Will keep the index of this gltf bone node aligned to irrlicht
    gltfToIrrlicht[thisBoneGltfIndex] = irrlichtJointIndex;

    // Now recurse it's children and add itself as their parent
    for (const int thisChild : gltfBoneNode.children) {
        recurseParentChildrenHierarchyInsertion(
            animatedMesh,
            model,
            thisSkin,
            thisChild,
            thisJoint,
            boneInsertionTracker,
            irrlichtToGltf,
            gltfToIrrlicht
        );
    }
}

void getWeightAndJointTargets(
    const tinygltf::Model model,
    std::map<std::string, int> attributes,
    const tinygltf::Skin thisSkin,

    int &jointVector4Count,

    tinygltf::Accessor &jointAccessor,
    tinygltf::BufferView &jointBufferView,
    tinygltf::Buffer &jointBuffer,
    int &jointByteStride,
    int &jointByteOffset,

    tinygltf::Accessor &weightAccessor,
    tinygltf::BufferView &weightBufferView,
    tinygltf::Buffer &weightBuffer,
    int &weightByteStride,
    int &weightByteOffset,

    tinygltf::Accessor &ibmAccessor,
    tinygltf::BufferView &ibmBufferView,
    tinygltf::Buffer &ibmBuffer,
    int &ibmByteStride,
    int &ibmByteOffset

    ) {
    // std::cout << "irr map size: " << irrlichtToGltf.size() << "\n";
    // std::cout << "gltf map size: " << gltfToIrrlicht.size() << "\n";

    // If this fails to parse the animation bone data, return a nullptr, this model is probably corrupted
    // if (addedJoints.size() == 0) {
    //     return nullptr;
    // }

    // Assign weight and joint (bone) target data to the mesh for this bone
    // joints count is synced
    // Weights are equal across all meshes. Here is a direct section of the gltf spec:
    /**
    To apply skinning, a transformation matrix is computed for each joint. Then, the per-vertex 
    transformation matrices are computed as weighted linear sums of the joint transformation matrices.
    Note that per-joint inverse bind matrices (when present) MUST be applied before the base node transforms.
    */


    const int jointAccessorId = attributes["JOINTS_0"];
    jointAccessor = model.accessors[jointAccessorId];
    jointVector4Count = jointAccessor.count;

    jointBufferView = model.bufferViews[jointAccessor.bufferView];
    jointBuffer = model.buffers[jointBufferView.buffer];
    jointByteStride = getByteStride(jointAccessor, jointBufferView);
    jointByteOffset = getByteOffset(jointAccessor, jointBufferView);

    const int weightAccessorId = attributes["WEIGHTS_0"];
    weightAccessor = model.accessors[weightAccessorId];
    weightBufferView = model.bufferViews[weightAccessor.bufferView];
    weightBuffer = model.buffers[weightBufferView.buffer];
    weightByteStride = getByteStride(weightAccessor, weightBufferView);
    weightByteOffset = getByteOffset(weightAccessor, weightBufferView);
    

    // Inverse Bind Matrices are REQUIRED in tinygltf - Shortening to IBM :P
    
    ibmAccessor = model.accessors[thisSkin.inverseBindMatrices];
    ibmBufferView = model.bufferViews[ibmAccessor.bufferView];

    ibmBuffer = model.buffers[ibmBufferView.buffer];
    ibmByteStride = getByteStride(ibmAccessor, ibmBufferView);
    ibmByteOffset = getByteOffset(ibmAccessor, ibmBufferView);
}


IAnimatedMesh* CGLTFMeshFileLoader::createMesh(io::IReadFile* file)
{
	tinygltf::Model model{};

	if (file->getSize() == 0 || !tryParseGLTF(file, model)) {
		return nullptr;
	}
    
    if (model.meshes.size() <= 0) {
        return nullptr;
    }

    /** 
        An animated mesh contains 2 very important elements:
        
        1. Mesh buffers, the primitives in gltf are their own buffer, they exist on their own outisde of other primitives.
            They have their own indices, texture coords, normals, and materials.
            This is how we can stack up different materials (textures) via
            textures = {"my_cool_texture_1.png", "something_awesome_2.png"}
            in the minetest api

        2. Animation data for each mesh buffer, irrlicht will automatically calculate frame time for each one!
            Each mesh buffer can independantly act in an animation frame :)
    */
    CSkinnedMesh* animatedMesh = new CSkinnedMesh();

    

    /*!
    //! TODO: THIS NEEDS TO BE FIGURED OUT

    An extremely important debugging

    Must figure out if bones are separated into meshes (model.mesh) OR
    are the bones separated into the primitives (model.meshes[id].primitives)

    This will be imperitive to deduce because it will allow any kind of model, not just single meshes
    to have animation data automatically implemented upon loading

    my test snowman is made up of 3 cubes, these are 3 meshes. The output data is:

    mesh index: 0 primitive: 0
    mesh index: 1 primitive: 0
    mesh index: 2 primitive: 0
    */
   

    // Iterate models
    for (int meshIndex = 0; meshIndex < model.meshes.size(); meshIndex++) {

        // Create a pointer cache so we don't have to keep iterating down the chain
        auto thisMesh = model.meshes[meshIndex];

        // Iterate primitives - Creates the model, vertex positions, tcoords, normals, indices
        for (int primitiveIndex = 0; primitiveIndex < thisMesh.primitives.size(); primitiveIndex++) {

            // TODO: test mesh animation with multiple primitives, might need a memory offset maybe

            // Create the base mesh
            scene::SSkinMeshBuffer *meshBuffer = animatedMesh->addMeshBuffer();
            // std::cout << "mesh index: " << meshIndex << " primitive: " << primitiveIndex << "\n";

            // Create a pointer cache so we don't have to keep iterating down the chain
            auto thisPrimitive = thisMesh.primitives[primitiveIndex];

            const int positionAccessorId = thisPrimitive.attributes["POSITION"];
            const int indicesAccessorId = thisPrimitive.indices;

            getIndices(model, indicesAccessorId, meshBuffer->Indices);
            getVertices(model, positionAccessorId, meshBuffer->Vertices_Standard, meshIndex, primitiveIndex);
            
            // We can't do any animation if we have no idea what the bone data is and these are synced
            if (!thisPrimitive.attributes.count("WEIGHTS_0") || !thisPrimitive.attributes.count("JOINTS_0")) {
                std::cout << "this model has no weight or joint data, it is now static\n";
                continue;
            }

            // No need to process animation if it's a static mesh
            if (model.animations.size() <= 0) {
                std::cout << "nodes: " << model.nodes.size() << "\n";
                std::cout << "mesh " << meshIndex << " primitive " << primitiveIndex << " does not have animation\n";
                continue;
            }
            

            // SKINS NEED TO BE AVAILABLE! Becomes a static model if it's not
            if (model.skins.size() <= 0) {
                std::cout << "I got no skin :(\n";
                continue;
            }

            // Create base animation data pointers & ref
            //* We are creating this here because we need to create the joints in the model before we assign weights to them
            /**
                Access objects of animation channels and samplers, these are synced.

                Channels are basically, different named animations. We don't care about that because minetest's
                API only allows you to index into animation channel 0 by floating point values.

                This is the (frame range) {x=0,y=1} in self.object:set_animation(frame range)

                There would need to be an additional accessor in set_animation that intakes a string, making it 3D.

                This is why we are doing: at.(0)
            */
            //! model.animations isn't a vector of animations per mesh, it's a vector of each animation like "run" "jump" yada yada
            // std::cout << "how many animations? " << model.animations.size() << "\n";
            const tinygltf::Animation thisAnimation = model.animations.at(0);

            // This is for translating gltf node index to irrlicht real joint index and vice versa
            std::map<int, int> gltfToIrrlicht;
            std::map<int, int> irrlichtToGltf;

            // std::cout << "SKIN AMOUNTS: " << model.skins.size() << "\n";

            // Have to assume skin 0, it's so insanely complex otherwise
            const auto thisSkin = model.skins.at(0);

            /**
                Recursive insertion - inserts parent-children hierarchy, we're using this
                to keep track of what we've already indexed. One parent can have multiple
                children but a child cannot have multiple parents!
            */
            std::map<int, int> boneInsertionTracker;

            // This starts off as a null bone, see line 164 of CSkinnedMesh.h and line 127 of CB3DMeshFileLoader.cpp
            /*
                //! Creates an array of joints from this mesh as children of node
                void addJoints(core::array<IBoneSceneNode*> &jointChildSceneNodes,
                IAnimatedMeshSceneNode* node,
                ISceneManager* smgr);
            */

           
            for (const int thisGltfBoneNode : thisSkin.joints) {
                recurseParentChildrenHierarchyInsertion(
                    animatedMesh,
                    model,
                    thisSkin,
                    thisGltfBoneNode,
                    (CSkinnedMesh::SJoint*)0,
                    boneInsertionTracker,
                    irrlichtToGltf,
                    gltfToIrrlicht
                );
            }
            

            // Add bone weights to the joints

            int jointVector4Count = 0;

            tinygltf::Accessor jointAccessor;
            tinygltf::BufferView jointBufferView;
            tinygltf::Buffer jointBuffer;
            int jointByteStride;
            int jointByteOffset;

            tinygltf::Accessor weightAccessor;
            tinygltf::BufferView weightBufferView;
            tinygltf::Buffer weightBuffer;
            int weightByteStride;
            int weightByteOffset;

            tinygltf::Accessor ibmAccessor;
            tinygltf::BufferView ibmBufferView;
            tinygltf::Buffer ibmBuffer;
            int ibmByteStride;
            int ibmByteOffset;
            
            getWeightAndJointTargets(
                model,
                thisPrimitive.attributes,
                thisSkin,

                jointVector4Count,

                jointAccessor,
                jointBufferView,
                jointBuffer,
                jointByteStride,
                jointByteOffset,

                weightAccessor,
                weightBufferView,
                weightBuffer,
                weightByteStride,
                weightByteOffset,

                ibmAccessor,
                ibmBufferView,
                ibmBuffer,
                ibmByteStride,
                ibmByteOffset
            );

            
            for (const int thisGltfBoneJointIndex : thisSkin.joints) {

                //  std::cout << thisGltfBoneJointIndex << " THIS is irr : " << gltfToIrrlicht[thisGltfBoneJointIndex] << " \n";

                const auto thisJointNode = model.nodes[thisGltfBoneJointIndex];

                CSkinnedMesh::SJoint *thisJoint = animatedMesh->getAllJoints()[gltfToIrrlicht[thisGltfBoneJointIndex]];

                // Add weights to the meshBuffer's indices
                for (int currentVec4 = 0; currentVec4 < jointVector4Count; currentVec4++) {
                    
                    const auto jointIndex = readBoneDataQuaternion(jointAccessor, BufferOffset( jointBuffer.data, jointByteOffset + (currentVec4 * jointByteStride)));
                    const auto currentWeight = readBoneDataQuaternion(weightAccessor, BufferOffset( weightBuffer.data, weightByteOffset + (currentVec4 * weightByteStride)));
                    
                    float jointArray[4] = {jointIndex.X, jointIndex.Y, jointIndex.Z, jointIndex.W};

                    float weightArray[4];

                    //! If the weights are in unsigned byte or unsigned short THEY ARE normalized
                    if (weightAccessor.componentType == 5121 || weightAccessor.componentType == 5123) {
                        // We need the values of the data's min and max, or denormalization is NOT possible
                        if (weightAccessor.minValues.size() == 0 || weightAccessor.maxValues.size() == 0) {
                            std::cout << "THERE IS NO NORMALIZATION DATA, THIS MODEL IS INVALID!\n";
                            return nullptr;
                        }

                        const float min = weightAccessor.minValues.at(0);
                        const float max = weightAccessor.maxValues.at(0);

                        weightArray[0] = (max - (float)currentWeight.X) / (max - min);
                        weightArray[1] = (max - (float)currentWeight.Y) / (max - min);
                        weightArray[2] = (max - (float)currentWeight.Z) / (max - min);
                        weightArray[3] = (max - (float)currentWeight.W) / (max - min);
                    } else {
                        weightArray[0] = currentWeight.X;
                        weightArray[1] = currentWeight.Y;
                        weightArray[2] = currentWeight.Z;
                        weightArray[3] = currentWeight.W;
                    }

                    
                    for (int w = 0; w < 4; w++) {

                        // Multiple primitives can mess with this
                        if (jointArray[w] >= thisSkin.joints.size()) {
                            continue;
                        }
                        
                        //* The joint vector is pointing to the skin's joint index!
                        const int fixed = thisSkin.joints.at(jointArray[w]);

                        if (thisGltfBoneJointIndex == fixed && weightArray[w] != 0.0) {

                            std::cout << "joint " << fixed << " influences indice " << currentVec4 << " with weight " << weightArray[w] << "\n";

                            CSkinnedMesh::SWeight *weight = animatedMesh->addWeight(thisJoint);

                            weight->buffer_id = meshIndex;
                            weight->vertex_id = currentVec4;
                            weight->strength = weightArray[w];

                        }
                    }
                }                
            }

            

            std::map<int, int> tracker;

            // Now that we have assigned all the translations to the bones
            for (const int gltfBoneNodeIndex : thisSkin.joints) {
                const core::matrix4 parentTransform = core::matrix4();
	            const core::matrix4 globalInverseWorldTransform = core::matrix4();
                recurseBoneTransforms(
                    tracker,
                    irrlichtToGltf,
                    gltfToIrrlicht,
                    model,
                    animatedMesh,
                    ibmBuffer,
                    ibmByteOffset,
                    ibmByteStride,
                    gltfBoneNodeIndex,
                    parentTransform,
                    globalInverseWorldTransform
                );
            }

           

            /**
             * Channels hold the target samplers for each animation
            */
            for (int currentChannelId = 0; currentChannelId < thisAnimation.channels.size(); currentChannelId++){

                const auto thisAnimationChannel = thisAnimation.channels.at(currentChannelId);


                const tinygltf::AnimationSampler thisAnimationSampler = thisAnimation.samplers.at(thisAnimationChannel.sampler);
                
                // Get which bone we're trying to animate
                const int gltfNodeBoneTarget = thisAnimationChannel.target_node;


                // std::cout << "TARGETING JOINT " << model.nodes[boneTarget].name << "\n";
                // std::cout << "JOINT " << boneTarget << " IS IRRLICHT " << gltfToIrrlicht(boneTarget) << "\n";

                CSkinnedMesh::SJoint *thisJoint = animatedMesh->getAllJoints()[gltfToIrrlicht[gltfNodeBoneTarget]];


                // std::cout << "this joint is: " << thisJoint->Name.c_str() << "\n";

                // Get the target path type - STRING (required in ["translation", "rotation", "scale", "weights"])
                const std::string animationTargetPath = thisAnimationChannel.target_path;
                
                

                //! Start keyframes (floating point, min to max)
                const int keyFrameAccessorIndex = thisAnimationSampler.input;
                const auto keyFrameAccessor = model.accessors[keyFrameAccessorIndex];
                //* Count is synced
                const int keyFrameCount = keyFrameAccessor.count;
                const auto keyFrameBufferView = model.bufferViews[keyFrameAccessor.bufferView];
                const auto keyFrameBuffer = model.buffers[keyFrameBufferView.buffer];
                
                const int keyFrameByteStride = getByteStride(keyFrameAccessor, keyFrameBufferView);
                const int keyFrameByteOffset = getByteOffset(keyFrameAccessor, keyFrameBufferView);

                //! Start data
                const int dataAccessorIndex = thisAnimationSampler.output;
                const auto dataAccessor = model.accessors[dataAccessorIndex];
                const auto dataBufferView = model.bufferViews[dataAccessor.bufferView];
                const auto dataBuffer = model.buffers[dataBufferView.buffer];

                const int dataByteStride = getByteStride(dataAccessor, dataBufferView);
                const int dataByteOffset = getByteOffset(dataAccessor, dataBufferView);

                // Now process it
                for (int currentFrame = 0; currentFrame < keyFrameCount; currentFrame++){
                    
                    //! TODO: FIXME: figure out how to make this disaster identical to blender frames
                    const float frameScalarValue = readPrimitive(keyFrameAccessor, BufferOffset( keyFrameBuffer.data, keyFrameByteOffset + (currentFrame * keyFrameByteStride)));

                    const float FRAME_RATE_IN_PROGRAM = 24.0;

                    const float currentFrameTime = frameScalarValue * FRAME_RATE_IN_PROGRAM;


                    // std::cout << "MAX FRAMES: " << MAX_FRAMES << "\n";

                    // const float currentFrameTime = MAX_FRAMES * ((MAX_FRAME_TIME - MIN_FRAME_TIME));

                    // std::cout << "currentFrameTime: " << currentFrameTime << "\n";

                    // validateValueFromAccessor(keyFrameAccessor, currentFrameTime);
                    
                    // std::cout << "my currentFrameTime is: "  << currentFrameTime << "\n";

                    if (animationTargetPath == "translation") {
                        irr::scene::ISkinnedMesh::SPositionKey *translation = animatedMesh->addPositionKey(thisJoint);
                        translation->frame = currentFrameTime;
                        translation->position = readVec3DF(BufferOffset( dataBuffer.data, dataByteOffset + (currentFrame * dataByteStride)) , core::vector3df(1));

                    } else if (animationTargetPath == "rotation") {
                        irr::scene::ISkinnedMesh::SRotationKey *rotation = animatedMesh->addRotationKey(thisJoint);
                        rotation->frame = currentFrameTime;
                        rotation->rotation = readQuaternion(BufferOffset( dataBuffer.data, dataByteOffset + (currentFrame * dataByteStride)) , core::vector3df(1));

                    } else if (animationTargetPath == "scale") {
                        irr::scene::ISkinnedMesh::SScaleKey *scale = animatedMesh->addScaleKey(thisJoint);
                        scale->frame = currentFrameTime;
                        scale->scale = readVec3DF(BufferOffset( dataBuffer.data, dataByteOffset + (currentFrame * dataByteStride)), core::vector3df(1));

                        //! Bolt on inversion test
                        // rotation->rotation *= core::vector3df(-1.0, -1.0, -1.0);
                    } else if (animationTargetPath == "weights") {
                        std::cout << "___--- THIS MODEL HAS WEIGHTS IN ANIMATION ----___\n";
                    }
                }
            }
        }
    }

	// Finish processing
    animatedMesh->finalize();

    // std::cout << "this animated mesh has " << animatedMesh->getAllJoints().size() << " bones within it!\n";
    // std::cout << "-------------\n";
    // std::string isStatic = animatedMesh->isStatic() ? "true" : "false";

    // std::cout << "root joint amount: " << animatedMesh->RootJoints.size() << "\n";

    // std::cout << "is this mesh static? " << isStatic << "\n";
    

    // //auto fps = animatedMesh->FramesPerSecond;
    //  auto frames = animatedMesh->getFrameCount();
    // // //std::cout << "FPS: " << fps << "\n";
    // std::cout << "animated mesh frame count: " << frames << "\n";

    // std::cout << "-------------\n";

    // std::cout << "endFrame = " << animatedMesh->EndFrame << "\n";

    return animatedMesh;
}

} // namespace scene

} // namespace irr