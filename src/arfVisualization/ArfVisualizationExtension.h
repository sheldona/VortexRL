#pragma once

#include <VxGraphics/IGraphic.h>
#include <VxGraphics/Gallery.h>
#include <VxSim/IDynamics.h>
#include <VxSim/IExtension.h>
#include <VxData/Container.h>
#include <VxData/Field.h>
#include <VxData/FieldArray.h>
#include <VxDynamics/Mechanism.h>
#include <VxSim/VxExtensionFactory.h>
#include <VxSim/VxUuid.h>
#include <Vx/VxID.h>

namespace arfVisualization
{
	const Vx::VxID kMechanismID("Mechanism");
	const VxSim::VxFactoryKey kExtensionFactoryKey(VxSim::VxUuid("4502C71D-A46F-46CE-9364-04B0FA441964"), "Arf", "Arf Collision Geometry Visualization Extension", "ArfVisualizationExtension");


	class ArfVisualizationExtension
		: public VxGraphics::IGraphic
		, public VxSim::IDynamics
		, public VxSim::IExtension
	{
	public:

		// Constructor
		ArfVisualizationExtension(VxSim::VxExtension* iProxy);

		// Destructor
		virtual ~ArfVisualizationExtension();

		virtual void onAddToSceneGraph(VxGraphics::SceneGraph*) override;

		VxData::Field < VxDynamics::Mechanism* > iMechanism;

		// TODO color input

	private:

		void initGallery();

		VxSim::VxSmartInterface<VxGraphics::Gallery> m_gallery;
	};
}
