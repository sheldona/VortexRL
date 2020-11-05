#pragma once

#include <Vx/VxArray.h>
#include <Vx/VxPart.h>

#include <VxSim/IKeyboard.h>
#include <VxSim/IExtension.h>
#include <VxSim/VxApplication.h>
#include <VxSim/VxSmartInterface.h>

#ifdef slots
#undef slots
#include <pybind11/numpy.h>
#define slots Q_SLOTS
#else
#include <pybind11/numpy.h>
#endif

namespace py = pybind11;

#include <tuple>

namespace Vx
{
	class VxUniverseState;
	class VxPrismatic;
	class VxHinge;
}

namespace VxGraphics
{
	class GraphicsModule;
}

class CartPoleEnvironment
{
public:
	CartPoleEnvironment(const std::string& filename, bool headless = false);

	~CartPoleEnvironment();

	// Steps the simulation  using the provided action. Returns the next state.
	//
	py::array_t<float> step(py::array_t<float> _action);

	// Resets the simulation to the initial state.
	//
	py::array_t<float> reset();


protected:
	// Initialize the app and build the gui.
	//
	void init(bool headless);

	// Update the state of the environment, stored in @a m_state
	//
	void updateState();
	

private:

	std::string m_filename;

	Vx::VxSmartPtr<VxSim::VxApplication> m_vxSimApp;
	Vx::VxSmartPtr<VxSim::VxExtension> m_dynamicsVisualizer;
	Vx::VxSmartPtr<VxSim::VxExtension> m_cartpoleVisualizer;

	Vx::VxUniverse* m_universe;
	Vx::VxUniverseState* m_initState;
	Vx::VxPrismatic* m_prismatic;
	Vx::VxHinge* m_hinge;
	Vx::VxPart* m_cart;

	float m_state[4];

public:

	// Keyboard extension class that listens for the keyboard input.
	class CartPoleKeyboardListener : public VxSim::IKeyboard, public VxSim::IExtension
	{
	public:
		// Constructor
		CartPoleKeyboardListener(VxSim::VxExtension* proxy);

		// Destructor
		virtual ~CartPoleKeyboardListener();

		// Sets the graphics module
		void setGraphicsModule(VxGraphics::GraphicsModule* gm);

		// Sets the application
		void setApplication(VxSim::VxApplication* app);

		// Called when key is pressed.
		// Displays feedback in the console of each camera.
		virtual void onKeyPressed(int key);

	private:
		VxSim::VxSmartInterface<VxGraphics::GraphicsModule> m_graphicsModule;

		VxSim::VxApplication* m_application;
	};

};

const VxSim::VxFactoryKey kCartPoleKeyboardListener(VxSim::VxUuid("{A615DD1A-4A66-4A35-848D-F3604F59E522}"), "arf", "CartPoleKeyboardListener");
