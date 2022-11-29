#include "camera.h"

#include <Expblas/graph_funcs.h>
#include <glad/glad.h>

using namespace Visual;

Camera::Camera(Expblas::vec3f position, Expblas::vec3f up, float yaw,
               float pitch)
    : Front(Expblas::vec3f(0.0f, 0.0f, -1.0f)), MovementSpeed(SPEED),
      MouseSensitivity(SENSITIVITY), Zoom(ZOOM) {
  Position = position;
  WorldUp = up;
  Yaw = yaw;
  Pitch = pitch;
  updateCameraVectors();
}

Camera::Camera(float posX, float posY, float posZ, float upX, float upY,
               float upZ, float yaw, float pitch)
    : Front(Expblas::vec3f(0.0f, 0.0f, -1.0f)), MovementSpeed(SPEED),
      MouseSensitivity(SENSITIVITY), Zoom(ZOOM) {
  Position = Expblas::vec3f(posX, posY, posZ);
  WorldUp = Expblas::vec3f(upX, upY, upZ);
  Yaw = yaw;
  Pitch = pitch;
  updateCameraVectors();
}

Expblas::mat4f Camera::GetViewMatrix() {
  return Expblas::lookAt(Position, Position + Front, Up);
}

void Camera::ProcessKeyboard(Camera_Movement direction, float deltaTime) {
  float velocity = MovementSpeed * deltaTime;
  if (direction == FORWARD)
    Position += velocity * Front;
  if (direction == BACKWARD)
    Position -= velocity * Front;
  if (direction == LEFT)
    Position -= velocity * Right;
  if (direction == RIGHT)
    Position += velocity * Right;
}

void Camera::ProcessMouseMovement(float xoffset, float yoffset,
                                  GLboolean constrainPitch) {
  xoffset *= MouseSensitivity;
  yoffset *= MouseSensitivity;

  Yaw += xoffset;
  Pitch += yoffset;

  // make sure that when pitch is out of bounds, screen doesn't get flipped
  if (constrainPitch) {
    if (Pitch > 89.0f)
      Pitch = 89.0f;
    if (Pitch < -89.0f)
      Pitch = -89.0f;
  }

  // update Front, Right and Up Vectors using the updated Euler angles
  updateCameraVectors();
}

void Camera::ProcessMouseScroll(float yoffset) {
  Zoom -= (float)yoffset;
  if (Zoom < 1.0f)
    Zoom = 1.0f;
  if (Zoom > 45.0f)
    Zoom = 45.0f;
}

void Camera::updateCameraVectors() {
  // calculate the new Front vector
  Expblas::vec3f front;
  front[0] = cos(radians(Yaw)) * cos(radians(Pitch));
  front[1] = sin(radians(Pitch));
  front[2] = sin(radians(Yaw)) * cos(radians(Pitch));
  Front = Expblas::normalized(front);
  // also re-calculate the Right and Up vector
  Right = Expblas::normalized(Expblas::cross(
      Front, WorldUp)); // normalize the vectors, because their length gets
                        // closer to 0 the more you look up or down which
                        // results in slower movement.
  Up = Expblas::normalized(Expblas::cross(Right, Front));
}