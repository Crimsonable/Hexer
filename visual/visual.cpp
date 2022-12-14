#define STB_IMAGE_STATIC
#define STB_IMAGE_IMPLEMENTATION
#include <stb_image.h>

#include "camera.h"
#include "meshLoader.h"
#include "model.h"
#include "shader.h"

#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>

#include <Expblas/graph_funcs.h>
#include <GLFW/glfw3.h>
#include <core/io.h>
#include <glad/glad.h>
#include <iostream>

void framebuffer_size_callback(GLFWwindow *window, int width, int height);
void mouse_callback(GLFWwindow *window, double xpos, double ypos);
void scroll_callback(GLFWwindow *window, double xoffset, double yoffset);
void processInput(GLFWwindow *window);

// settings
const unsigned int SCR_WIDTH = 800;
const unsigned int SCR_HEIGHT = 600;

// camera
Visual::Camera camera(Expblas::vec3f(0.0f, 0.0f, 3.0f));
float lastX = SCR_WIDTH / 2.0f;
float lastY = SCR_HEIGHT / 2.0f;
bool firstMouse = true;

// timing
float deltaTime = 0.0f;
float lastFrame = 0.0f;

int main() {
  // glfw: initialize and configure
  // ------------------------------
  glfwInit();
  glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
  glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
  glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);

#ifdef __APPLE__
  glfwWindowHint(GLFW_OPENGL_FORWARD_COMPAT, GL_TRUE);
#endif

  // glfw window creation
  // --------------------
  GLFWwindow *window =
      glfwCreateWindow(SCR_WIDTH, SCR_HEIGHT, "LearnOpenGL", NULL, NULL);
  if (window == NULL) {
    std::cout << "Failed to create GLFW window" << std::endl;
    glfwTerminate();
    return -1;
  }
  glfwMakeContextCurrent(window);
  glfwSetFramebufferSizeCallback(window, framebuffer_size_callback);
  glfwSetCursorPosCallback(window, mouse_callback);
  glfwSetScrollCallback(window, scroll_callback);

  // tell GLFW to capture our mouse
  glfwSetInputMode(window, GLFW_CURSOR, GLFW_CURSOR_DISABLED);

  // glad: load all OpenGL function pointers
  // ---------------------------------------
  if (!gladLoadGLLoader((GLADloadproc)glfwGetProcAddress)) {
    std::cout << "Failed to initialize GLAD" << std::endl;
    return -1;
  }

  // tell stb_image.h to flip loaded texture's on the y-axis (before loading
  // model).
  stbi_set_flip_vertically_on_load(true);

  // configure global opengl state
  // -----------------------------
  glEnable(GL_DEPTH_TEST);

  // build and compile shaders

  // Visual::gShaderMap["pure_color"] =
  //     Visual::Shader("../../../../visual/shader/basic_model.vs",
  //                    "../../../../visual/shader/pure_color.fs");

  // Visual::gShaderMap["texture"] =
  //     Visual::Shader("../../../../visual/shader/basic_model.vs",
  //                    "../../../../visual/shader/basic_model.fs");

  Visual::gShaderMap["pure_color"] =
      Visual::Shader("A:/MeshGeneration/Hexer/visual/shader/basic_model.vs",
                     "A:/MeshGeneration/Hexer/visual/shader/pure_color.fs");

  Visual::gShaderMap["texture"] =
      Visual::Shader("A:/MeshGeneration/Hexer/visual/shader/basic_model.vs",
                     "A:/MeshGeneration/Hexer/visual/shader/basic_model.fs");
  // load models
  // -----------
  Hexer::PolyMeshReader reader;
  auto mesh = reader.execute("A:/MeshGeneration/OpenVolumeMesh/examples/"
                             "vtk_datafile_2/vtk_files/s01c_cube.vtk");
  auto model = Visual::convertMeshToModel(mesh);

  // Visual::Model ourModel(
  //     "D:/codes/LearnOpenGL/resources/objects/backpack/backpack.obj");

  // draw in wireframe
  auto &shader = Visual::gShaderMap["pure_color"];
  // render loop
  // -----------
  while (!glfwWindowShouldClose(window)) {
    // per-frame time logic
    // --------------------
    float currentFrame = static_cast<float>(glfwGetTime());
    deltaTime = currentFrame - lastFrame;
    lastFrame = currentFrame;

    // input
    // -----
    processInput(window);

    // render
    // ------
    glClearColor(0.05f, 0.05f, 0.05f, 1.0f);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    // don't forget to enable shader before setting uniforms
    shader.use();

    // view/projection transformations
    auto projection = Expblas::perspective(Visual::radians(camera.Zoom),
                                           (float)SCR_WIDTH / (float)SCR_HEIGHT,
                                           0.1f, 100.0f);
    Expblas::mat4f view = camera.GetViewMatrix();
    shader.setMat4("projection", projection);
    shader.setMat4("view", view);

    // render the loaded model
    Expblas::mat4f model_mat =
        Expblas::eye<float, Expblas::StorageMajor::ColumnMajor, 4>();
    // model = Expblas::translate(model, glm::vec3(0.0f, 0.0f, 0.0f)); //
    // translate it down so it's at the center of the scene model =
    // glm::scale(model, glm::vec3(1.0f, 1.0f, 1.0f));	// it's a bit too big
    // for our scene, so scale it down
    shader.setMat4("model", model_mat);

    // glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
    // shader.setVec4("pureColor", Expblas::vec4f(1, 1, 1, 1));
    // model.Draw(shader);

    glPolygonMode(GL_FRONT, GL_FILL);
    shader.setVec4("pureColor", Expblas::vec4f(0, 1, 0, 1));
    model.Draw(shader);

    // glfw: swap buffers and poll IO events (keys pressed/released, mouse moved
    // etc.)
    // -------------------------------------------------------------------------------
    glfwSwapBuffers(window);
    glfwPollEvents();
  }

  // glfw: terminate, clearing all previously allocated GLFW resources.
  // ------------------------------------------------------------------
  glfwTerminate();
  return 0;
}

// process all input: query GLFW whether relevant keys are pressed/released this
// frame and react accordingly
// ---------------------------------------------------------------------------------------------------------
void processInput(GLFWwindow *window) {
  if (glfwGetKey(window, GLFW_KEY_ESCAPE) == GLFW_PRESS)
    glfwSetWindowShouldClose(window, true);

  if (glfwGetKey(window, GLFW_KEY_W) == GLFW_PRESS)
    camera.ProcessKeyboard(Visual::FORWARD, deltaTime);
  if (glfwGetKey(window, GLFW_KEY_S) == GLFW_PRESS)
    camera.ProcessKeyboard(Visual::BACKWARD, deltaTime);
  if (glfwGetKey(window, GLFW_KEY_A) == GLFW_PRESS)
    camera.ProcessKeyboard(Visual::LEFT, deltaTime);
  if (glfwGetKey(window, GLFW_KEY_D) == GLFW_PRESS)
    camera.ProcessKeyboard(Visual::RIGHT, deltaTime);
}

// glfw: whenever the window size changed (by OS or user resize) this callback
// function executes
// ---------------------------------------------------------------------------------------------
void framebuffer_size_callback(GLFWwindow *window, int width, int height) {
  // make sure the viewport matches the new window dimensions; note that width
  // and height will be significantly larger than specified on retina displays.
  glViewport(0, 0, width, height);
}

// glfw: whenever the mouse moves, this callback is called
// -------------------------------------------------------
void mouse_callback(GLFWwindow *window, double xposIn, double yposIn) {
  float xpos = static_cast<float>(xposIn);
  float ypos = static_cast<float>(yposIn);

  if (firstMouse) {
    lastX = xpos;
    lastY = ypos;
    firstMouse = false;
  }

  float xoffset = xpos - lastX;
  float yoffset =
      lastY - ypos; // reversed since y-coordinates go from bottom to top

  lastX = xpos;
  lastY = ypos;

  camera.ProcessMouseMovement(xoffset, yoffset);
}

// glfw: whenever the mouse scroll wheel scrolls, this callback is called
// ----------------------------------------------------------------------
void scroll_callback(GLFWwindow *window, double xoffset, double yoffset) {
  camera.ProcessMouseScroll(static_cast<float>(yoffset));
}