#include "ArfVisualizationExtension.h"

#include <VxGraphics/GraphicsModule.h>
#include <VxGraphics/GraphicsEnums.h>
#include <VxContent/Scene.h>
#include <VxGraphics/Gallery.h>
#include <VxGraphics/Image.h>
#include <VxGraphics/Texture.h>
#include <VxGraphics/Material.h>
#include <VxGraphics/Mesh.h>
#include <VxGraphics/MeshHelper.h>
#include <VxGraphics/Node.h>
#include <VxGraphics/NodeHelper.h>
#include <VxGraphics/ObjectHelper.h>
#include <VxGraphics/Services.h>
#include <VxGraphics/ShapeGenerator.h>
#include <VxGraphics/ShapeMeshBuilder.h>
#include <VxGraphics/ShadowManager.h>

#include <VxGraphicsPlugins/GraphicsModuleICD.h>
#include <VxGraphicsPlugins/GalleryICD.h>
#include <VxGraphicsPlugins/NodeICD.h>
#include <VxGraphicsPlugins/GeometryICD.h>
#include <VxGraphicsPlugins/NodeICD.h>
#include <VxGraphicsPlugins/MaterialICD.h>
#include <VxContent/ConnectionContainerExtensionICD.h>
#include <VxContent/ConnectionContainerExtension.h>


#include <VxDynamics/Mechanism.h>
#include <VxDynamics/CollisionGeometry.h>
#include <VxDynamics/Part.h>
#include <Vx/VxUniverse.h>
#include <Vx/VxPart.h>
#include <Vx/VxCollisionGeometry.h>
#include <Vx/VxSphere.h>
#include <Vx/VxCapsule.h>
#include <Vx/VxBox.h>
#include <VxMath/Transformation.h>

#include <VxSim/VxApplication.h>

using namespace arfVisualization;


ArfVisualizationExtension::ArfVisualizationExtension(VxSim::VxExtension* iProxy) :
	VxGraphics::IGraphic(iProxy), VxSim::IExtension(iProxy), VxSim::IDynamics(iProxy),
	iMechanism(nullptr, kMechanismID, &iProxy->getInputContainer()),
	m_gallery()
	
{
	iMechanism.addObserver([this](const VxData::Field<VxDynamics::Mechanism*>&) {
        initGallery();
	});

}

ArfVisualizationExtension::~ArfVisualizationExtension()
{

}

void ArfVisualizationExtension::onAddToSceneGraph(VxGraphics::SceneGraph*)
{

}

void ArfVisualizationExtension::initGallery()
{
    if (iMechanism.getValue() == nullptr)
        return;

    // TODO: clear existing nodes if the mechanism changes

    m_gallery = getGraphicsModule()->getApplication()->getSimulationFileManager()->loadObject("../../resources/ShapeGenerator.vxgraphicgallery");
    getGraphicsModule()->getApplication()->remove(m_gallery);
    iMechanism->getProxy()->add(m_gallery->getProxy());

    VxSim::VxSmartInterface<VxGraphics::Material> defaultMaterial = VxSim::VxExtensionFactory::create(VxGraphicsPlugins::MaterialICD::kFactoryKey);
    auto* layer = &(defaultMaterial->mAlbedoLayers.front());
    layer->mColor.setValue(Vx::VxColor(0, 0, 1, 1));
    defaultMaterial->setName("defaultMaterial");
    m_gallery->getProxy()->add(defaultMaterial->getProxy());

    VxSim::VxSmartInterface<VxContent::ConnectionContainerExtension> connections = VxSim::VxExtensionFactory::create(VxContent::ConnectionContainerExtensionICD::kFactoryKey);
    iMechanism->getProxy()->add(connections->getProxy());

    for (const auto assembly : iMechanism->getAssemblies())
    {
        for (const auto part : assembly->getParts())
        {
            Vx::VxPart* p = part->getVxPart();
            const auto& geoms = p->getCollisionGeometries();

            VxSim::VxSmartInterface<VxGraphics::Node> node = VxSim::VxExtensionFactory::create(VxGraphicsPlugins::NodeICD::kFactoryKey);
            node->setName(p->getName());
            m_gallery->getProxy()->add(node->getProxy());

            for (const auto collGeom : part->getCollisionGeometries())
            {
                VxSim::VxSmartInterface<VxGraphics::Node> cgNode = VxSim::VxExtensionFactory::create(VxGraphicsPlugins::NodeICD::kFactoryKey);

                Vx::VxCollisionGeometry* cg = collGeom->getVxCollisionGeometry();

                if (Vx::VxSphere* sphere = dynamic_cast<Vx::VxSphere*>(cg->getGeometry()))
                {
                    VxSim::VxSmartInterface <VxGraphics::Geometry> sphereGeometry = VxGraphics::ObjectHelper::findGeometry(m_gallery, "Sphere_Geometry");
                    if (sphereGeometry.valid())
                    {
                        auto* node_lod = &(cgNode->inputLODs.front());
                        node_lod->inputGeometry = sphereGeometry;
                        node_lod->inputMaterial = defaultMaterial;
                        node_lod->inputMaximumDistance = 1000.0;
                        node_lod->inputCastShadow = true;

                        const double radius = sphere->getRadius();
						const VxMath::Vector3 sphereScale(2.0 * radius, 2.0 * radius, 2.0 * radius);
                        VxMath::Matrix44 sphereTransform = collGeom->inputLocalTransform;
                        VxMath::Transformation::scale(sphereTransform, sphereScale);
                        cgNode->inputLocalTransform = sphereTransform;
                    }
                }
                else if (Vx::VxBox* box = dynamic_cast<Vx::VxBox*>(cg->getGeometry()))
                {
                    VxSim::VxSmartInterface <VxGraphics::Geometry> boxGeometry = VxGraphics::ObjectHelper::findGeometry(m_gallery, "Box_Geometry");
                    if (boxGeometry.valid())
                    {
                        auto* node_lod = &(cgNode->inputLODs.front());
                        node_lod->inputGeometry = boxGeometry;
                        node_lod->inputMaterial = defaultMaterial;
                        node_lod->inputMaximumDistance = 1000.0;
                        node_lod->inputCastShadow = true; 

                        Vx::VxReal3 dim;
                        box->getDimensions(dim);

                        VxMath::Matrix44 boxTransform = collGeom->inputLocalTransform;
                        VxMath::Transformation::scale(boxTransform, dim);
                        cgNode->inputLocalTransform = boxTransform;
                    }
                }
                else if (Vx::VxCapsule* capsule = dynamic_cast<Vx::VxCapsule*>(cg->getGeometry()))
                {
                    const double radius = capsule->getRadius();
                    const double length = capsule->getLength();
                    const VxMath::Vector3 pipeScale(2 * radius, 2 * radius, length);
                    const VxMath::Vector3 capScale(2 * radius, 2 * radius, 2 * radius);
                    VxSim::VxSmartInterface <VxGraphics::Geometry> pipeGeometry = VxGraphics::ObjectHelper::findGeometry(m_gallery, "Pipe_Geometry");
                    VxSim::VxSmartInterface <VxGraphics::Geometry> capTop = VxGraphics::ObjectHelper::findGeometry(m_gallery, "CapTop_Geometry");
                    VxSim::VxSmartInterface <VxGraphics::Geometry> capBot = VxGraphics::ObjectHelper::findGeometry(m_gallery, "CapBottom_Geometry");

                    VxSim::VxSmartInterface<VxGraphics::Node> pipeNode = VxSim::VxExtensionFactory::create(VxGraphicsPlugins::NodeICD::kFactoryKey);
                    {
                        VxMath::Matrix44 pipeTransform = VxMath::Transformation::createScale(pipeScale);
                        auto* node_lod = &(pipeNode->inputLODs.front());
                        node_lod->inputGeometry = pipeGeometry;
                        node_lod->inputMaterial = defaultMaterial;
                        node_lod->inputMaximumDistance = 1000.0;
                        node_lod->inputCastShadow = true;
                        pipeNode->inputLocalTransform = pipeTransform;
                    }
                    VxSim::VxSmartInterface<VxGraphics::Node> capTopNode = VxSim::VxExtensionFactory::create(VxGraphicsPlugins::NodeICD::kFactoryKey);
                    {
                        auto* node_lod = &(capTopNode->inputLODs.front());
                        node_lod->inputGeometry = capTop;
                        node_lod->inputMaterial = defaultMaterial;
                        node_lod->inputMaximumDistance = 1000.0;
                        node_lod->inputCastShadow = true;

                        VxMath::Matrix44 topCapTransform = VxMath::Transformation::createTranslation(0.0, 0.0, length / 2.0);
                        VxMath::Transformation::scale(topCapTransform, capScale);
                        capTopNode->inputLocalTransform = topCapTransform;
                    }
                    VxSim::VxSmartInterface<VxGraphics::Node> capBotNode = VxSim::VxExtensionFactory::create(VxGraphicsPlugins::NodeICD::kFactoryKey);
                    {
                        auto* node_lod = &(capBotNode->inputLODs.front());
                        node_lod->inputGeometry = capBot;
                        node_lod->inputMaterial = defaultMaterial;
                        node_lod->inputMaximumDistance = 1000.0;
                        node_lod->inputCastShadow = true;

                        VxMath::Matrix44 bottomCapTransform = VxMath::Transformation::createTranslation(0.0, 0.0, -length / 2.0);
                        VxMath::Transformation::scale(bottomCapTransform, capScale);
                        capBotNode->inputLocalTransform.setValue(bottomCapTransform);
                    }

                    cgNode->getProxy()->add(pipeNode->getProxy());
                    cgNode->getProxy()->add(capTopNode->getProxy());
                    cgNode->getProxy()->add(capBotNode->getProxy());

                    VxMath::Matrix44 capsuleTransform = collGeom->inputLocalTransform;
                    cgNode->inputLocalTransform = capsuleTransform;
                }
                connections->create(&part->outputWorldTransform, &node->inputParentTransform);
                node->getProxy()->add(cgNode->getProxy());
            }

        }
    }

    // Enable shadow casting for all nodes in the gallery.
    VxGraphics::NodeHelper::enableNodesCastShadow(m_gallery.getInterface(), true);

    VxGraphics::ShadowManager& shadow = getGraphicsModule()->getServices()->getShadowManager();

    // SA: for some reasons, shadows don't work unless we do this???
    const size_t numShadowMaps = shadow.getNumberofShadowMaps();
    shadow.enable();
}
