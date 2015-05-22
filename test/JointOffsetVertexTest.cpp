/*
 * JointOffsetVertexTest.cpp
 *
 *  Created on: 08.12.2013
 *      Author: stefan
 */

#include "../include/optimization/JointOffsetVertex.h"
#include "../include/common/KinematicChain.h"

#include <gtest/gtest.h>

namespace kinematic_calibration {

class KinematicChainMock: public KinematicChain {
public:
	KinematicChainMock(){}
	virtual void getJointNames(vector<string>& jointNames) const {
		jointNames.push_back("first");
		jointNames.push_back("second");
		jointNames.push_back("third");
	}
};
// ----------------------------------------------------------------------------
// SingleJointOffsetVertex
// ----------------------------------------------------------------------------

TEST(SingleJointOffsetVertex, oplusTest) {
	// Arrange
	string name = "testee";
	SingleJointOffsetVertex* testee = new SingleJointOffsetVertex(name);
	testee->setToOrigin();
	double delta[1];
    delta[0] = 1.23;

	// Act
	testee->oplus(delta);
	double value = testee->estimate();

	// Assert
	double eps = 1e-6;
	ASSERT_NEAR(value, delta[0], eps);
}

TEST(SingleJointOffsetVertex, setEstimateTest) {
	// Arrange
	string name = "testee";
	SingleJointOffsetVertex* testee = new SingleJointOffsetVertex(name);
	double estimate = 1.23;

	// Act
	testee->setEstimate(estimate);
	double value = testee->estimate();

	// Assert
	double eps = 1e-6;
	ASSERT_NEAR(value, estimate, eps);
}

TEST(SingleJointOffsetVertex, setToOriginTest) {
	// Arrange
	string name = "testee";
	SingleJointOffsetVertex* testee = new SingleJointOffsetVertex(name);
	testee->setEstimate(1.23);

	// Act
	testee->setToOrigin();
	double value = testee->estimate();

	// Assert
	double eps = 1e-6;
	ASSERT_NEAR(value, 0.0, eps);
}

// ----------------------------------------------------------------------------
// JointOffsetVertexContainer
// ----------------------------------------------------------------------------

TEST(JointOffsetVertexContainer, addVertexForJointTest) {
	// Arrange
	string first = "first";
	string second = "second";
	string third = "third";
	JointOffsetVertexContainer container;
	container.addVertexForJoint(first);
	container.addVertexForJoint(second);
	container.addVertexForJoint(third);

	// Act
	map<string, SingleJointOffsetVertex*> vertices = container.getAllVertices();
	int numOfVertices = vertices.size();

	// Assert
	ASSERT_EQ(3, numOfVertices);
}

TEST(JointOffsetVertexContainer, addVerticesForChainTest) {
	// Arrange
	string first = "first";
	string second = "second";
	string third = "third";
	KinematicChainMock chain;
	JointOffsetVertexContainer container;
	container.addVerticesForChain(chain, true, true);

	// Act
	map<string, SingleJointOffsetVertex*> vertices = container.getAllVertices();
	int numOfVertices = vertices.size();
	SingleJointOffsetVertex* firstVertex = vertices[first];
	SingleJointOffsetVertex* secondVertex = vertices[second];
	SingleJointOffsetVertex* thirdVertex = vertices[third];

	// Assert
	ASSERT_EQ(3, numOfVertices);
	ASSERT_TRUE(firstVertex != NULL);
	ASSERT_TRUE(firstVertex->fixed());
	ASSERT_TRUE(secondVertex != NULL);
	ASSERT_FALSE(secondVertex->fixed());
	ASSERT_TRUE(thirdVertex != NULL);
	ASSERT_TRUE(thirdVertex->fixed());
}

TEST(JointOffsetVertexContainer, getAllVerticesTest) {
	// Arrange
	string first = "first";
	string second = "second";
	string third = "third";
	JointOffsetVertexContainer container;
	container.addVertexForJoint(first);
	container.addVertexForJoint(second);
	container.addVertexForJoint(third);

	// Act
	map<string, SingleJointOffsetVertex*> vertices = container.getAllVertices();
	SingleJointOffsetVertex* firstVertex = vertices[first];
	SingleJointOffsetVertex* nonexistentVertex = vertices["bla"];

	// Assert
	ASSERT_TRUE(NULL != firstVertex);
	ASSERT_TRUE(NULL == nonexistentVertex);
}

TEST(JointOffsetVertexContainer, fixVertexTest) {
	// Arrange
	string first = "first";
	string second = "second";
	string third = "third";
	JointOffsetVertexContainer container;
	container.addVertexForJoint(first, false);
	container.addVertexForJoint(second, true);
	container.addVertexForJoint(third);

	// Act
	container.fixVertex(third);
	map<string, SingleJointOffsetVertex*> vertices = container.getAllVertices();
	SingleJointOffsetVertex* firstVertex = vertices[first];
	SingleJointOffsetVertex* secondVertex = vertices[second];
	SingleJointOffsetVertex* thirdVertex = vertices[third];

	// Assert
	ASSERT_FALSE(firstVertex->fixed());
	ASSERT_TRUE(secondVertex->fixed());
	ASSERT_TRUE(thirdVertex->fixed());
}

TEST(JointOffsetVertexContainer, updateJointOffsetTest) {
	// Arrange
	string first = "first";
	JointOffsetVertexContainer container;
	container.addVertexForJoint(first);

	// Act
	double offset = 2.34;
	container.updateJointOffset(first, offset);

	map<string, double> jointOffsets = container.getJointOffsets();
	bool jointOffsetExists = jointOffsets.count(first);
	double value = jointOffsets[first];

	// Assert
	double eps = 1e-6;
	ASSERT_TRUE(jointOffsetExists);
	ASSERT_NEAR(offset, value, eps);
}

TEST(JointOffsetVertexContainer, addMimicJointTest1) {
	// Arrange
	string first = "first";
	string second = "second";
	JointOffsetVertexContainer container;
	container.setMimicJoint(first, second);
	container.addVertexForJoint(first);
	container.addVertexForJoint(second);

	// Act
	map<string, SingleJointOffsetVertex*> vertices = container.getAllVertices();

	// Assert
	ASSERT_TRUE(vertices.size() == 2);
	ASSERT_TRUE(vertices.count(first) == 1);
	ASSERT_TRUE(vertices.count(second) == 1);
	ASSERT_FALSE(vertices[first]->fixed());
	ASSERT_TRUE(vertices[second]->fixed());
}

TEST(JointOffsetVertexContainer, addMimicJointTest2) {
	// Arrange
	string first = "first";
	string second = "second";
	JointOffsetVertexContainer container;
	container.setMimicJoint(first, second);
	container.addVertexForJoint(second);

	// Act
	map<string, SingleJointOffsetVertex*> vertices = container.getAllVertices();

	// Assert
	ASSERT_TRUE(vertices.size() == 1);
	ASSERT_TRUE(vertices.count(first) == 0);
	ASSERT_TRUE(vertices.count(second) == 1);
	ASSERT_FALSE(vertices[second]->fixed());
}

TEST(JointOffsetVertexContainer, addMimicJointTest3) {
	// Arrange
	string first = "first";
	string second = "second";
	JointOffsetVertexContainer container;
	container.setMimicJoint(first, second);
	container.addVertexForJoint(second);
	container.addVertexForJoint(first);

	// Act
	map<string, SingleJointOffsetVertex*> vertices = container.getAllVertices();

	// Assert
	ASSERT_TRUE(vertices.size() == 2);
	ASSERT_TRUE(vertices.count(first) == 1);
	ASSERT_TRUE(vertices.count(second) == 1);
	ASSERT_FALSE(vertices[first]->fixed());
	ASSERT_TRUE(vertices[second]->fixed());
}

TEST(JointOffsetVertexContainer, getVerticesForChainTest) {
		// Arrange
		string first = "first";
		string second = "second";
		string third = "third";
		string another = another;
		KinematicChainMock chain;
		JointOffsetVertexContainer container;
		container.addVerticesForChain(chain, false, false);
		container.addVertexForJoint(another);

		// Act
		map<string, SingleJointOffsetVertex*> vertices = container.getVerticesForChain(chain);
		int numOfVertices = vertices.size();
		SingleJointOffsetVertex* firstVertex = vertices[first];
		SingleJointOffsetVertex* secondVertex = vertices[second];
		SingleJointOffsetVertex* thirdVertex = vertices[third];

		// Assert
		ASSERT_EQ(3, numOfVertices);
		ASSERT_TRUE(firstVertex != NULL);
		ASSERT_FALSE(firstVertex->fixed());
		ASSERT_TRUE(secondVertex != NULL);
		ASSERT_FALSE(secondVertex->fixed());
		ASSERT_TRUE(thirdVertex != NULL);
		ASSERT_FALSE(thirdVertex->fixed());
	}
// ----------------------------------------------------------------------------
// SingleJointOffsetVertex and JointOffsetVertexContainer (integration)
// ----------------------------------------------------------------------------
TEST(JointOffsetVertexContainer, integrationTest) {
	// Arrange
	string first = "first";
	JointOffsetVertexContainer container;
	container.addVertexForJoint(first);

	// Act
	double offset = 2.34;
	SingleJointOffsetVertex* vertex = container.getAllVertices()[first];
	vertex->oplus(&offset);

	map<string, double> jointOffsets = container.getJointOffsets();
	bool jointOffsetExists = jointOffsets.count(first);
	double value = jointOffsets[first];

	// Assert
	double eps = 1e-6;
	ASSERT_TRUE(jointOffsetExists);
	ASSERT_NEAR(offset, value, eps);
}

} /* namespace kinematic_calibration */

// Run all the tests that were declared with TEST()
int main(int argc, char **argv) {
	testing::InitGoogleTest(&argc, argv);
	return RUN_ALL_TESTS();
}
