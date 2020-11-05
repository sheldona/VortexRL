#include "CartPoleEnvironment.h"
#include "ArfVisualizationExtension.h"

#include <Vx/VxArray.h>
#include <Vx/VxAssembly.h>
#include <Vx/VxConstraint.h>
#include <Vx/VxDistance.h>
#include <Vx/VxDynamicsContact.h>
#include <Vx/VxHinge.h>
#include <Vx/VxPrismatic.h>
#include <VxContent/Scene.h>
#include <VxDynamics/Constraint.h>
#include <VxDynamics/Mechanism.h>
#include <VxDynamics/Part.h>

#include <VxSim/VxApplication.h>
#include <VxSim/VxSmartInterface.h>
#include <VxSim/VxDynamicsModuleICD.h>
#include <VxSim/VxApplicationMode.h>
#include <VxSim/VxExtensionFactory.h>
#include <Vx/VxUniverse.h>
#include <Vx/VxUniverseState.h>

#include <VxGraphics/GraphicsModule.h>
#include <VxGraphics/ICamera.h>
#include <VxGraphicsPlugins/GraphicsModuleICD_OSG.h>
#include <VxGraphicsPlugins/DisplayICD.h>
#include <VxGraphicsPlugins/DynamicsVisualizerICD.h>
#include <VxGraphicsPlugins/PerspectiveICD.h>
#include <VxGraphicsPlugins/ViewCameraManipulatorICD.h>

#include <functional>
#include <iostream>
#include <random>
#include <tuple>

using namespace Vx;
using namespace VxContent;
using namespace VxDynamics;
using namespace VxSim;

namespace
{
	static const int s_stateDim = 4;

	// Returns the first instance of a part with name @a partName
	VxPart* findPartByName(VxDynamics::Mechanism* mechanism, const std::string& name)
	{
		for (auto a : mechanism->getAssemblies())
		{
			for (auto p : a->getParts())
			{
				if (p->getName() == name)
					return p->getVxPart();
			}
		}
		return nullptr;
	}

	// Returns the first instance of a part with name @a partName
	VxConstraint* findConstraintByName(VxDynamics::Mechanism* mechanism, const std::string& name)
	{
		for (auto a : mechanism->getAssemblies())
		{
			for (auto c : a->getConstraints())
			{
				if (c->getName() == name)
					return c->getVxConstraint();
			}
		}
		return nullptr;
	}
}

CartPoleEnvironment::CartPoleKeyboardListener::CartPoleKeyboardListener(VxSim::VxExtension* proxy) :
	VxSim::IKeyboard(proxy),
	VxSim::IExtension(proxy),
	m_graphicsModule(),
	m_application(nullptr)
{

}

CartPoleEnvironment::CartPoleKeyboardListener::~CartPoleKeyboardListener() { }

void CartPoleEnvironment::CartPoleKeyboardListener::setGraphicsModule(VxGraphics::GraphicsModule* gm)
{
	m_graphicsModule = gm;
}

void CartPoleEnvironment::CartPoleKeyboardListener::setApplication(VxSim::VxApplication* app)
{
	m_application = app;
}


void CartPoleEnvironment::CartPoleKeyboardListener::onKeyPressed(int key)
{
	//
	// Do stuff here related to keyboard presses
	//

	// Enable vsyncing
	if (key == 'f')
	{
		m_graphicsModule->getProxy()->getParameter(VxGraphicsPlugins::GraphicsModuleICD::kEnableFrameLimiter)->setValue(true);
		m_application->setSyncMode(VxSim::kSyncVSync);
	}
	// Free run / no frame syncing
	else if (key == 'r')
	{
		m_graphicsModule->getProxy()->getParameter(VxGraphicsPlugins::GraphicsModuleICD::kEnableFrameLimiter)->setValue(false);
		m_application->setSyncMode(VxSim::kSyncNone);
	}
}


CartPoleEnvironment::CartPoleEnvironment(const std::string& filename, bool headless) :
	m_filename(filename),
	m_vxSimApp(),
	m_dynamicsVisualizer(),
	m_cartpoleVisualizer(),
	m_initState(nullptr),
	m_universe(nullptr)
{
	VxSim::VxExtensionFactory::registerType<CartPoleEnvironment::CartPoleKeyboardListener>(kCartPoleKeyboardListener, "");

	init(headless);
}

CartPoleEnvironment::~CartPoleEnvironment()
{
	VxSim::VxExtensionFactory::unregisterType(kCartPoleKeyboardListener);
}

void CartPoleEnvironment::updateState()
{
	m_state[0] = m_cart->getPosition().y();		// cart horizontal position
	m_state[1] = m_cart->getLinearVelocity().y();	// cart linear velocity
	m_state[2] = m_hinge->recalculateCoordinateCurrentPosition(VxHinge::kAngularCoordinate);	// hinge rotation
	m_state[3] = m_hinge->recalculateCoordinateVelocity(VxHinge::kAngularCoordinate);		// hinge angular velocity
}

py::array_t<float> CartPoleEnvironment::reset()
{
	m_universe->restoreState(m_initState);
	m_universe->resetDynamics();
	m_universe->resetContacts();

	updateState();

	auto st = py::array_t<float>(s_stateDim, m_state);
	return st;
}

py::array_t<float> CartPoleEnvironment::step(py::array_t<float> _action)
{
	// Convert the action to a float pointer.
	float* action_ptr = static_cast<float*>(_action.request().ptr);
	const float vel = *action_ptr;

	m_prismatic->setMotorDesiredVelocity(VxPrismatic::kLinearCoordinate, vel);
	m_vxSimApp->update();
	m_vxSimApp->update();
	m_vxSimApp->update();
	m_vxSimApp->update();

	updateState();

	return py::array_t<float>(s_stateDim, m_state);
}

void CartPoleEnvironment::init(bool headless)
{
	// Create application
	m_vxSimApp = new VxSim::VxApplication;
	auto dynamicsModule = VxSim::VxSmartInterface<VxSim::ISimulatorModule>::create(VxSim::VxDynamicsModuleICD::kFactoryKey);
	m_vxSimApp->insertModule(dynamicsModule);
	m_vxSimApp->setSyncMode(VxSim::kSyncNone);
	m_vxSimApp->setApplicationMode(VxSim::kModeSimulating);
	m_vxSimApp->setSimulationFrameRate(60.0);
	m_vxSimApp->enableConsoleLog(false);

	if (!headless)
	{
		auto graphicsModule = VxSim::VxExtensionFactory::create(VxGraphicsPlugins::GraphicsModuleICD::kModuleFactoryKey);
		graphicsModule->getParameter(VxGraphicsPlugins::GraphicsModuleICD::kEnablePicking)->setValue(true);
		graphicsModule->getParameter(VxGraphicsPlugins::GraphicsModuleICD::kEnableSSAO)->setValue(false);
		graphicsModule->getParameter(VxGraphicsPlugins::GraphicsModuleICD::kAntiAliasing)->setValue(false);
		graphicsModule->getParameter(VxGraphicsPlugins::GraphicsModuleICD::kAnisotropy)->setValue(false);
		VxSim::VxSmartInterface<VxSim::ISimulatorModule> graphicsSimulatorModule = graphicsModule;
		m_vxSimApp->insertModule(graphicsSimulatorModule);

		Vx::VxSmartPtr<VxSim::VxExtension> display = VxSim::VxExtensionFactory::create(VxGraphicsPlugins::DisplayICD::kExtensionFactoryKey);
		if (!display.valid())
		{
			Vx::LogWarn("Could not load create Display extension");
		}
		display->setName("Vx simulation");
		display->getInput(VxGraphicsPlugins::IDisplayAutomaticPlacementItemICD::kPlacementMode)->setValue(std::string("Windowed"));
		display->getInput(VxGraphicsPlugins::IDisplayAutomaticPlacementItemICD::kPlacement)->setValue(VxMath::Vector4(50, 50, 640, 480));
		m_vxSimApp->add(display.get());
		m_dynamicsVisualizer = VxSim::VxExtensionFactory::create(VxGraphicsPlugins::DynamicsVisualizerICD::kExtensionFactoryKey);
		m_vxSimApp->add(m_dynamicsVisualizer);
		m_dynamicsVisualizer->getInput(VxGraphicsPlugins::DynamicsVisualizerICD::kDisplayCollisionGeometry)->setValue(false);
		m_dynamicsVisualizer->getInput(VxGraphicsPlugins::DynamicsVisualizerICD::kDisplayContact)->setValue(false);

		// Hand visualization
		m_cartpoleVisualizer = VxSim::VxExtensionFactory::create(arfVisualization::kExtensionFactoryKey);
		m_vxSimApp->add(m_cartpoleVisualizer);

		// Add a free camera
		Vx::VxSmartPtr<VxSim::VxExtension> freeCameraExt = VxSim::VxExtensionFactory::create(VxGraphicsPlugins::PerspectiveICD::kExtensionFactoryKey);
		VxSim::VxWeakInterface<VxGraphics::ICamera> freeCamera(freeCameraExt);
		freeCamera->lookAt(Vx::VxVector3(-10, 0, 5), Vx::VxVector3(0, 0, 0));
		m_vxSimApp->add(freeCameraExt.get());

		// Add the camera manipulator
		Vx::VxSmartPtr<VxSim::VxExtension> viewCameraManipulator = VxSim::VxExtensionFactory::create(VxGraphicsPlugins::ViewCameraManipulatorICD::kExtensionFactoryKey);
		m_vxSimApp->add(viewCameraManipulator.get());

		// Add custom keyboard listener
		Vx::VxSmartPtr<VxSim::VxExtension> keyboardListenerExtension = VxSim::VxExtensionFactory::create(kCartPoleKeyboardListener);
		VxSim::VxWeakInterface<CartPoleKeyboardListener> listener(keyboardListenerExtension);
		if (listener.valid())
		{
			listener->setGraphicsModule(VxSim::VxWeakInterface<VxGraphics::GraphicsModule>(graphicsModule));
			listener->setApplication(m_vxSimApp.get());
			m_vxSimApp->add(keyboardListenerExtension.get());
		}

	}

	// Load the test scene
	VxSim::VxSmartInterface<VxContent::Scene> scene = m_vxSimApp->getSimulationFileManager()->loadObject(m_filename);

	// Find the cartpole mechanism
	//
	auto mechanisms = scene->getMechanisms();

	Vx::VxAssert(mechanisms.size() > 0, "Must have at least one mechanism.");

	// Cartpole is the first mechanism
	VxSim::VxSmartInterface<VxDynamics::Mechanism> cartpoleMechanism = mechanisms[0];

	if ( !headless )
	{
		// Hand visualization
		VxData::FieldBase* fieldBase = m_cartpoleVisualizer->getInput("Mechanism");
		fieldBase->setValue(cartpoleMechanism.getExtension());
	}

	m_prismatic = dynamic_cast<Vx::VxPrismatic*>(findConstraintByName(cartpoleMechanism, "Prismatic"));
	Vx::VxAssert(m_prismatic != nullptr, "Could not find prismatic joint.");

	m_hinge = dynamic_cast<Vx::VxHinge*>(findConstraintByName(cartpoleMechanism, "Hinge"));
	Vx::VxAssert(m_hinge != nullptr, "Could not find hinge joint.");

	m_cart = findPartByName(cartpoleMechanism, "BasePart");
	Vx::VxAssert(m_cart != nullptr, "Could not cart part.");

	// Store a pointer to the universe.
	m_universe = m_cart->getUniverse();

	m_vxSimApp->update();
	m_initState = m_universe->createState();
}
