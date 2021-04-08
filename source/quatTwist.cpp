// ---------------------------------------------------------------------
//
//  quatTwist.cpp
//
//  Created by ingo on 10/10/16.
//  Copyright (c) 2021 Ingo Clemens. All rights reserved.
//
// ---------------------------------------------------------------------

#include <string>

static const std::string kVERSION = "2.2.0";

#include <iostream>
#include <cmath>

#include <maya/MAngle.h>
#include <maya/MAnimControl.h>
#include <maya/MArrayDataBuilder.h>
#include <maya/MDagPath.h>
#include <maya/MDataBlock.h>
#include <maya/MDataHandle.h>
#include <maya/MFnAnimCurve.h>
#include <maya/MFnCompoundAttribute.h>
#include <maya/MFnEnumAttribute.h>
#include <maya/MFnMatrixAttribute.h>
#include <maya/MFnNumericAttribute.h>
#include <maya/MFnPlugin.h>
#include <maya/MFnUnitAttribute.h>
#include <maya/MGlobal.h>
#include <maya/MMatrix.h>
#include <maya/MPlug.h>
#include <maya/MPlugArray.h>
#include <maya/MPxNode.h>
#include <maya/MQuaternion.h>
#include <maya/MRampAttribute.h>
#include <maya/MTransformationMatrix.h>
#include <maya/MVector.h>
#include <maya/MTypeId.h>

#ifdef _WIN64
#define M_PI 3.1415926535897932384626433832795
#endif

const float RADTODEG = (float)(180 / M_PI);


// ---------------------------------------------------------------------
// header
// ---------------------------------------------------------------------

class quatTwist : public MPxNode
{
public:
    quatTwist();
    virtual ~quatTwist();

    static void* creator();
    static MStatus initialize();

    void postConstructor();
    MStatus postConstructor_init_curveRamp(MObject &nodeObj,
                                           MObject &rampObj,
                                           int index,
                                           float position,
                                           float value,
                                           int interpolation);

    MStatus compute(const MPlug &plug,
                    MDataBlock &data);

    double getTwistAngle(MQuaternion quat,
                         int axis);
    double trackRotation(double value,
                         double reference);
    bool isAnimated(MPlug plug,
                    MPlug &animPlug,
                    bool &isConnected);

    static MTypeId id;

private:
    static MStatus affectsOutput(MObject &attr);

public:
    // attributes
    static MObject active;
    static MObject axis;
    static MObject inputMatrix;
    static MObject invertTwist;
    static MObject offset;
    static MObject restMatrix;
    static MObject reverseSegments;
    static MObject scale;
    static MObject segmentBlend;
    static MObject segmentCount;
    static MObject twist;
    static MObject twistSegmentOut;
    static MObject useCurve;

    // rotation tracking
    static MObject autoRegulate;
    static MObject extend;
    static MObject poseVector;
    static MObject regulationAngle;
    static MObject transformIndex;
    static MObject twistInput;
    static MObject twistReference;
};


// ---------------------------------------------------------------------
// main
// ---------------------------------------------------------------------

MTypeId quatTwist::id(0x0011C1C8);

MObject quatTwist::active;
MObject quatTwist::autoRegulate;
MObject quatTwist::axis;
MObject quatTwist::extend;
MObject quatTwist::inputMatrix;
MObject quatTwist::invertTwist;
MObject quatTwist::offset;
MObject quatTwist::poseVector;
MObject quatTwist::regulationAngle;
MObject quatTwist::restMatrix;
MObject quatTwist::reverseSegments;
MObject quatTwist::scale;
MObject quatTwist::segmentBlend;
MObject quatTwist::transformIndex;
MObject quatTwist::twist;
MObject quatTwist::twistInput;
MObject quatTwist::twistReference;
MObject quatTwist::twistSegmentOut;
MObject quatTwist::useCurve;
MObject quatTwist::segmentCount;


// ---------------------------------------------------------------------
// creator
// ---------------------------------------------------------------------

quatTwist::quatTwist()
{}

quatTwist::~quatTwist()
{}

void* quatTwist::creator()
{
    return new quatTwist();
}


// ---------------------------------------------------------------------
// initialize the attributes
// ---------------------------------------------------------------------

MStatus quatTwist::initialize()
{
    //
    // MFnEnumAttribute
    //

    MFnEnumAttribute eAttr;

    axis = eAttr.create("axis", "ax", 0);
    eAttr.addField("X", 0);
    eAttr.addField("Y", 1);
    eAttr.addField("Z", 2);

    //
    // MFnNumericAttribute
    //

    MFnNumericAttribute nAttr;

    active = nAttr.create("active", "ac", MFnNumericData::kBoolean);
    nAttr.setKeyable(true);
    nAttr.setDefault(true);

    segmentCount = nAttr.create("segmentCount", "sc", MFnNumericData::kLong);
    nAttr.setKeyable(true);
    nAttr.setMin(1);
    nAttr.setDefault(1);

    invertTwist = nAttr.create("invertTwist", "it", MFnNumericData::kBoolean);
    nAttr.setKeyable(true);
    nAttr.setDefault(false);

    offset = nAttr.create("offset", "of", MFnNumericData::kDouble);
    nAttr.setKeyable(true);
    nAttr.setDefault(0.0);
    nAttr.setSoftMin(-90.0);
    nAttr.setSoftMax(90.0);

    scale = nAttr.create("scale", "s", MFnNumericData::kDouble);
    nAttr.setKeyable(true);
    nAttr.setDefault(1.0);
    nAttr.setSoftMin(0.0);
    nAttr.setSoftMax(2.0);

    reverseSegments = nAttr.create("reverseSegments", "rs", MFnNumericData::kBoolean);
    nAttr.setKeyable(true);
    nAttr.setDefault(false);

    useCurve = nAttr.create("useCurve", "uc", MFnNumericData::kBoolean);
    nAttr.setKeyable(true);
    nAttr.setDefault(false);

    extend = nAttr.create("extend", "ext", MFnNumericData::kBoolean);
    nAttr.setKeyable(false);
    nAttr.setHidden(true);
    nAttr.setDefault(true);

    poseVector = nAttr.create("poseVector", "pv", MFnNumericData::k3Double);
    nAttr.setHidden(true);

    autoRegulate = nAttr.create("autoRegulate", "ar", MFnNumericData::kBoolean);
    nAttr.setKeyable(true);
    nAttr.setDefault(true);

    regulationAngle = nAttr.create("regulationAngle", "ra", MFnNumericData::kDouble);
    nAttr.setKeyable(false);
    nAttr.setHidden(false);
    nAttr.setDefault(45.0);

    transformIndex = nAttr.create("transformIndex", "tid", MFnNumericData::kLong);
    nAttr.setKeyable(true);
    nAttr.setMin(0);
    nAttr.setDefault(0);

    //
    // MFnUnitAttribute
    //

    MFnUnitAttribute uAttr;

    twist = uAttr.create("twist", "tw", MFnUnitAttribute::kAngle);
    uAttr.setKeyable(false);
    uAttr.setDefault(0.0);

    twistSegmentOut = uAttr.create("twistSegmentOut", "tso", MFnUnitAttribute::kAngle);
    uAttr.setKeyable(false);
    uAttr.setArray(true);
    uAttr.setUsesArrayDataBuilder(true);

    twistInput = uAttr.create("twistInput", "ti", MFnUnitAttribute::kAngle);
    uAttr.setKeyable(false);
    uAttr.setArray(true);
    uAttr.setUsesArrayDataBuilder(true);

    twistReference = uAttr.create("twistReference", "tr", MFnUnitAttribute::kAngle);
    uAttr.setKeyable(false);
    uAttr.setHidden(true);
    uAttr.setDefault(0.0);

    //
    // MFnMatrixAttribute
    //

    MFnMatrixAttribute mAttr;

    inputMatrix = mAttr.create("inputMatrix", "im");
    mAttr.setHidden(true);

    restMatrix = mAttr.create("restMatrix", "rm");
    mAttr.setHidden(true);

    //
    // MRampAttribute
    //

    MRampAttribute rAttr;

    segmentBlend = rAttr.createCurveRamp("segmentBlend", "sb");

    // -----------------------------------------------------------------
    // add attributes (order matters)
    // -----------------------------------------------------------------

    addAttribute(active);
    addAttribute(inputMatrix);
    addAttribute(restMatrix);
    addAttribute(axis);
    addAttribute(twist);
    addAttribute(segmentCount);
    addAttribute(invertTwist);
    addAttribute(offset);
    addAttribute(scale);
    addAttribute(reverseSegments);
    addAttribute(useCurve);
    addAttribute(segmentBlend);
    addAttribute(twistSegmentOut);
    addAttribute(extend);
    addAttribute(poseVector);
    addAttribute(twistInput);
    addAttribute(transformIndex);
    addAttribute(twistReference);
    addAttribute(autoRegulate);
    addAttribute(regulationAngle);

    // -----------------------------------------------------------------
    // affects
    // -----------------------------------------------------------------

    CHECK_MSTATUS_AND_RETURN_IT(affectsOutput(active));
    CHECK_MSTATUS_AND_RETURN_IT(affectsOutput(axis));
    CHECK_MSTATUS_AND_RETURN_IT(affectsOutput(extend));
    CHECK_MSTATUS_AND_RETURN_IT(affectsOutput(inputMatrix));
    CHECK_MSTATUS_AND_RETURN_IT(affectsOutput(invertTwist));
    CHECK_MSTATUS_AND_RETURN_IT(affectsOutput(offset));
    CHECK_MSTATUS_AND_RETURN_IT(affectsOutput(restMatrix));
    CHECK_MSTATUS_AND_RETURN_IT(affectsOutput(reverseSegments));
    CHECK_MSTATUS_AND_RETURN_IT(affectsOutput(scale));
    CHECK_MSTATUS_AND_RETURN_IT(affectsOutput(segmentBlend));
    CHECK_MSTATUS_AND_RETURN_IT(affectsOutput(segmentCount));
    CHECK_MSTATUS_AND_RETURN_IT(affectsOutput(useCurve));

    return MS::kSuccess;
}


MStatus quatTwist::affectsOutput(MObject &attr)
{
    CHECK_MSTATUS_AND_RETURN_IT(attributeAffects(attr, quatTwist::twist));
    CHECK_MSTATUS_AND_RETURN_IT(attributeAffects(attr, quatTwist::twistSegmentOut));

    return MStatus::kSuccess;
}


void quatTwist::postConstructor()
{
    MObject thisNode = this->thisMObject();

    // One entry is the least needed or the attribute editor will
    // produce an error.
    postConstructor_init_curveRamp(thisNode, segmentBlend, 0, 0.0f, 0.0f, 1);
    postConstructor_init_curveRamp(thisNode, segmentBlend, 1, 1.0f, 1.0f, 1);
}


MStatus quatTwist::postConstructor_init_curveRamp(MObject &nodeObj,
                                                  MObject &rampObj,
                                                  int index,
                                                  float position,
                                                  float value,
                                                  int interpolation)
{
    MStatus status;

    MPlug rampPlug(nodeObj, rampObj);
    MPlug elementPlug = rampPlug.elementByLogicalIndex((unsigned)index, &status);
    CHECK_MSTATUS_AND_RETURN_IT(status);
    MPlug positionPlug = elementPlug.child(0, &status);
    CHECK_MSTATUS_AND_RETURN_IT(status);
    status = positionPlug.setFloat(position);
    CHECK_MSTATUS_AND_RETURN_IT(status);
    MPlug valuePlug = elementPlug.child(1);
    status = valuePlug.setFloat(value);
    CHECK_MSTATUS_AND_RETURN_IT(status);
    MPlug interpolationPlug = elementPlug.child(2);
    interpolationPlug.setInt(interpolation);

    return MS::kSuccess;
}


// ---------------------------------------------------------------------
// compute
// ---------------------------------------------------------------------

MStatus quatTwist::compute(const MPlug &plug,
                           MDataBlock &data)
{
    MStatus status = MS::kSuccess;

    MObject thisNode = this->thisMObject();

    // -----------------------------------------------------------------
    // attributes
    // -----------------------------------------------------------------

    MDataHandle autoRegulateData = data.inputValue(autoRegulate, &status);
    CHECK_MSTATUS_AND_RETURN_IT(status);
    bool regulateVal = autoRegulateData.asBool();

    MDataHandle activeData = data.inputValue(active, &status);
    CHECK_MSTATUS_AND_RETURN_IT(status);
    bool activeVal = activeData.asBool();

    MDataHandle axisData = data.inputValue(axis, &status);
    CHECK_MSTATUS_AND_RETURN_IT(status);
    int axisVal = axisData.asShort();

    MDataHandle extendData = data.inputValue(extend, &status);
    CHECK_MSTATUS_AND_RETURN_IT(status);
    bool extendVal = extendData.asBool();

    MDataHandle invertTwistData = data.inputValue(invertTwist, &status);
    CHECK_MSTATUS_AND_RETURN_IT(status);
    bool invertVal = invertTwistData.asBool();

    MDataHandle offsetData = data.inputValue(offset, &status);
    CHECK_MSTATUS_AND_RETURN_IT(status);
    double offsetVal = offsetData.asDouble();

    MDataHandle regulationAngleData = data.inputValue(regulationAngle, &status);
    CHECK_MSTATUS_AND_RETURN_IT(status);
    double regulationAngleVal = regulationAngleData.asDouble();

    MDataHandle reverseSegmentsData = data.inputValue(reverseSegments, &status);
    CHECK_MSTATUS_AND_RETURN_IT(status);
    bool reverseVal = reverseSegmentsData.asBool();

    MDataHandle scaleData = data.inputValue(scale, &status);
    CHECK_MSTATUS_AND_RETURN_IT(status);
    double scaleVal = scaleData.asDouble();

    MDataHandle segmentCountData = data.inputValue(segmentCount, &status);
    CHECK_MSTATUS_AND_RETURN_IT(status);
    int countVal = segmentCountData.asShort();

    MDataHandle transformIndexData = data.inputValue(transformIndex, &status);
    CHECK_MSTATUS_AND_RETURN_IT(status);
    int transformIndexVal = transformIndexData.asShort();

    MDataHandle twistReferenceData = data.inputValue(twistReference, &status);
    CHECK_MSTATUS_AND_RETURN_IT(status);
    double twistRefVal = twistReferenceData.asAngle().asDegrees();

    MDataHandle useCurveData = data.inputValue(useCurve, &status);
    CHECK_MSTATUS_AND_RETURN_IT(status);
    bool useCurveVal = useCurveData.asBool();

    MRampAttribute blendAttr = MRampAttribute(thisNode, segmentBlend, &status);
    CHECK_MSTATUS_AND_RETURN_IT(status);

    MDataHandle poseDataHandle = data.inputValue(poseVector, &status);
    CHECK_MSTATUS_AND_RETURN_IT(status);
    double3& poseVal = poseDataHandle.asDouble3();

    // -----------------------------------------------------------------
    // plugs
    // -----------------------------------------------------------------

    MPlug matrixPlug(thisNode, quatTwist::inputMatrix);
    MPlug twistInputPlug(thisNode, quatTwist::twistInput);
    MPlug twistPlug(thisNode, quatTwist::twist);
    MPlug twistReferencePlug(thisNode, quatTwist::twistReference);

    // -----------------------------------------------------------------
    // check if the node should be computed
    // -----------------------------------------------------------------

    // Deactivate the node if the state is set to HasNoEffect.
    MDataHandle stateData = data.inputValue(state, &status);
    if (stateData.asShort() == 1)
        return status;

    // -----------------------------------------------------------------
    // get the input twist reference
    // -----------------------------------------------------------------

    MIntArray twistInputIds;

    twistInputPlug.getExistingArrayAttributeIndices(twistInputIds, &status);
    CHECK_MSTATUS_AND_RETURN_IT(status);

    MArrayDataHandle twistInputHandle = data.inputArrayValue(twistInput, &status);
    CHECK_MSTATUS_AND_RETURN_IT(status);

    unsigned int inputCount = twistInputIds.length();

    MPlug animPlug;
    bool isConnected = false;

    if (inputCount > 0)
    {
        // Check if the current transform index matches one of the
        // existing attribute ids.
        bool exists = false;
        for (unsigned int i = 0; i < inputCount; i ++)
        {
            if (twistInputIds[i] == transformIndexVal)
            {
                exists = true;
                break;
            }
        }

        if (exists)
        {
            MPlug inPlug = twistInputPlug.connectionByPhysicalIndex((unsigned)transformIndexVal);

            // If the input is animated replace the internal twist
            // reference with the twist input value.
            // This also returns the attribute on the transform node and
            // if the twist input index is actually connected.
            // This is important for setting the twist value on the
            // transform node.
            bool animated = isAnimated(inPlug, animPlug, isConnected);
            if (animated)
            {
                status = twistInputHandle.jumpToArrayElement((unsigned)transformIndexVal);
                CHECK_MSTATUS_AND_RETURN_IT(status);
                twistRefVal = twistInputHandle.inputValue().asAngle().asDegrees();
            }
        }
    }

    // -----------------------------------------------------------------
    // get the input and rest matrix data
    // -----------------------------------------------------------------

    MDataHandle matrixHandle = data.inputValue(inputMatrix, &status);
    CHECK_MSTATUS_AND_RETURN_IT(status);
    MMatrix inputMat = matrixHandle.asMatrix();

    MDataHandle restHandle = data.inputValue(restMatrix, &status);
    CHECK_MSTATUS_AND_RETURN_IT(status);
    MMatrix restMat = restHandle.asMatrix();

    // -----------------------------------------------------------------
    // get the parent matrix
    // -----------------------------------------------------------------

    MPlugArray plugConn;
    matrixPlug.connectedTo(plugConn, true, false, &status);
    CHECK_MSTATUS_AND_RETURN_IT(status);
    // cancel, if the matrix is not connected
    if (!plugConn.length())
        return status;
    MDagPath dagPath;
    MDagPath::getAPathTo(plugConn[0].node(), dagPath);
    MMatrix parentMatInv = dagPath.exclusiveMatrixInverse();

    // -----------------------------------------------------------------
    // get the quaternions
    // -----------------------------------------------------------------

    MTransformationMatrix inputTransMat = inputMat * parentMatInv;
    MQuaternion quatInput = inputTransMat.rotation();

    MTransformationMatrix restTransMat = restMat;
    MQuaternion quatRest = restTransMat.rotation();

    // -----------------------------------------------------------------
    // main calculation
    // -----------------------------------------------------------------

    MQuaternion quat;
    MVector pose(1.0, 0.0, 0.0);

    double twistVal = 0.0;

    if (activeVal)
    {
        // Build the relative quaternion.
        quat = quatInput * quatRest.conjugate();

        // The current issue with tracking the rotation is that larger
        // changes to the current orientation can lead to false results
        // due to the twist reference value which is mandatory for the
        // tracking to work. For example, setting the joint back to the
        // default pose from large rotation values usually leaves the
        // twist at 360. To reduce this problem a delta angle between
        // the current pose vector and the previous pose gets
        // calculated. Usually during animation or manual interaction
        // this angle is rather small but if rotation changes with
        // larger values occur this angle gets larger as well.
        // If the given angle (default 45 degrees) is exceeded the
        // reference value gets modified/reduced to a remainder of 180,
        // which should be sufficient in most cases.
        if (extendVal && regulateVal)
        {
            // Define the reference vector base.
            if (axisVal == 1)
                pose = MVector(0.0, 1.0, 0.0);
            else if (axisVal == 2)
                pose = MVector(0.0, 0.0, 1.0);

            // Transform the pose based on the current orientation.
            pose *= inputTransMat.asMatrix();

            // Compare with the last pose and modify the reference value
            // if needed.
            double delta = pose.angle(MVector(poseVal[0], poseVal[1], poseVal[2])) * RADTODEG;
            if (delta > regulationAngleVal)
                twistRefVal = fmod(twistRefVal, 180.0);
        }

        twistVal = getTwistAngle(quat, axisVal);
        if (invertVal)
            twistVal *= -1;

        if (extendVal)
            twistVal = trackRotation(twistVal, twistRefVal);
    }

    // -----------------------------------------------------------------
    // pass the pose value to the output
    // -----------------------------------------------------------------

    // twist
    data.outputValue(twistPlug).set(MAngle((twistVal * scaleVal) + offsetVal, MAngle::kDegrees));
    data.outputValue(twistReferencePlug).set(MAngle(twistVal, MAngle::kDegrees));

    // pose vector
    data.outputValue(poseVector).set3Double(pose.x, pose.y, pose.z);

    // Set the twist reference on the transform node.
    if (isConnected)
    {
        MAngle animAngle(twistVal, MAngle::kDegrees);
        animPlug.setMAngle(animAngle);
    }

    // -----------------------------------------------------------------
    // segments
    // -----------------------------------------------------------------

    // twist segment
    MArrayDataHandle twistHandle = data.outputArrayValue(twistSegmentOut);
    MArrayDataBuilder twistBuilder(&data, twistSegmentOut, (unsigned)countVal);

    // Use different cases for non-curve-based and curve-based segment.
    // Though it duplicates some parts it's easier to manage the
    // different approaches to calculate the segments.
    if (!useCurveVal)
    {
        double twistSubVal = twistVal * scaleVal * (1.0 / countVal);
        double twistInc = twistSubVal;
        if (reverseVal)
        {
            twistInc = twistVal * scaleVal;
            twistSubVal *= -1;
        }

        for (unsigned int i = 0; i < (unsigned)countVal; i ++)
        {
            MDataHandle twistIdHandle = twistBuilder.addElement(i);
            twistIdHandle.set(MAngle(twistInc + offsetVal, MAngle::kDegrees));
            twistHandle.set(twistBuilder);
            twistHandle.setAllClean();

            twistInc += twistSubVal;
        }
    }
    else
    {
        float subStep = (float)(1.0 / countVal);

        for (unsigned int i = 0; i < (unsigned)countVal; i ++)
        {
            int stepMult = (int)(i + 1);
            if (reverseVal)
                stepMult = countVal - (int)i;

            float curveVal;
            blendAttr.getValueAtPosition(subStep * stepMult, curveVal);
            double subVal = twistVal * curveVal * scaleVal;

            MDataHandle twistIdHandle = twistBuilder.addElement(i);
            twistIdHandle.set(MAngle(subVal + offsetVal, MAngle::kDegrees));
            twistHandle.set(twistBuilder);
            twistHandle.setAllClean();
        }
    }

    data.setClean(plug);

    return MStatus::kSuccess;
}


//
// Description:
//      Calculate the twist angle based on the given rotate order.
//
// Input Arguments:
//      q                   The rotation quaternion.
//      axis                The rotation axis.
//
// Return Value:
//      double              The twist angle.
//
double quatTwist::getTwistAngle(MQuaternion q,
                                int axis)
{
    double axisComponent = q.x;
    if (axis == 1)
        axisComponent = q.y;
    else if (axis == 2)
        axisComponent = q.z;
    return RADTODEG * (2.0 * atan2(axisComponent, q.w));
}


//
// Description:
//      Return the twist value based on the given reference angle.
//
// Input Arguments:
//      twistValue          The current twist value.
//      reference           The reference angle.
//
// Return Value:
//      double              The expanded twist angle.
//
double quatTwist::trackRotation(double twistValue,
                                double reference)
{
    // Get the relative twist based on the current angle and the last
    // angle.
    double relativeTwist = reference - twistValue;

    // Expand the angle range beyond 360 degrees based on the reference
    // value.
    int cycle = 0;
    remquo(relativeTwist, 360.0, &cycle);
    twistValue = twistValue + 360 * cycle;

    return twistValue;
}


//
// Description:
//      Return if the given plug is animated.
//
// Input Arguments:
//      plug                The MPlug to check for an animation.
//      animPlug            The source MPlug of the connection.
//      isConnected         Returns true, if the plug is connected.
//
// Return Value:
//      bool                True, if the given plug is animated.
//
bool quatTwist::isAnimated(MPlug plug,
                           MPlug &animPlug,
                           bool &isConnected)
{
    MStatus status = MS::kSuccess;

    bool animated = false;

    MPlugArray plugArray;

    // First check if the current twistInput plug is connected (to a
    // transform node)
    plug.connectedTo(plugArray, true, false, &status);
    CHECK_MSTATUS_AND_RETURN_IT(status);

    if (plugArray.length())
    {
        isConnected = true;
        animPlug = plugArray[0];

        // Check if the quatTwist attribute on the transform node is
        // driven by an animation curve.
        plugArray[0].connectedTo(plugArray, true, false, &status);
        CHECK_MSTATUS_AND_RETURN(status, false);

        if (plugArray.length())
        {
            MObject animObj = plugArray[0].node();
            if (animObj.hasFn(MFn::kAnimCurve))
            {
                MFnAnimCurve animFn(animObj);

                unsigned int numKeys = animFn.numKeys();
                MTime keyTimeFirst = animFn.time(0, &status);
                CHECK_MSTATUS_AND_RETURN(status, false);
                MTime keyTimeLast = animFn.time(numKeys - 1, &status);
                CHECK_MSTATUS_AND_RETURN(status, false);

                MTime currentTime = MAnimControl::currentTime();

                if (currentTime >= keyTimeFirst && currentTime <= keyTimeLast)
                    animated = true;
            }
        }
    }
    return animated;
}


// ---------------------------------------------------------------------
// initialization
// ---------------------------------------------------------------------

MStatus initializePlugin(MObject obj)
{
    MStatus status;
    MFnPlugin plugin(obj, "Ingo Clemens", kVERSION.c_str(), "Any");

    status = plugin.registerNode("quatTwist",
                                 quatTwist::id,
                                 quatTwist::creator,
                                 quatTwist::initialize);
    if (status != MStatus::kSuccess)
        status.perror("Register quatTwist node failed");

    return status;
}

MStatus uninitializePlugin(MObject obj)
{
    MStatus status;
    MFnPlugin plugin(obj, "Ingo Clemens", kVERSION.c_str(), "Any");

    status = plugin.deregisterNode(quatTwist::id);

    if (status != MStatus::kSuccess)
        status.perror("Deregister quatTwist node failed");

    return status;
}

// ---------------------------------------------------------------------
// MIT License
//
// Copyright (c) 2021 Ingo Clemens, brave rabbit
// quatTwist is under the terms of the MIT License
//
// Permission is hereby granted, free of charge, to any person obtaining
// a copy of this software and associated documentation files (the
// "Software"), to deal in the Software without restriction, including
// without limitation the rights to use, copy, modify, merge, publish,
// distribute, sublicense, and/or sell copies of the Software, and to
// permit persons to whom the Software is furnished to do so, subject to
// the following conditions:
//
// The above copyright notice and this permission notice shall be
// included in all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
// EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
// MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.
// IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY
// CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT,
// TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE
// SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
//
// Author: Ingo Clemens    www.braverabbit.com
// ---------------------------------------------------------------------
