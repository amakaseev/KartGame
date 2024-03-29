using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.Assertions.Must;

namespace Kart {

  [System.Serializable]
  public class AxleInfo {
    public WheelCollider leftWheel;
    public WheelCollider rightWheel;
    public bool motor;
    public bool steering;
    public WheelFrictionCurve originalForwardFriction;
    public WheelFrictionCurve originalSidewaysFriction;
  }

  public class KartController : MonoBehaviour {

    [Header("Axle Information")]
    [SerializeField] AxleInfo[] axleInfos;

    [Header("Motor Attributs")]
    [SerializeField] float maxMotorTorque = 3000f;
    [SerializeField] float maxSpeed;

    [Header("Steering Attributes")]
    [SerializeField] float maxSteeringAngle = 30f;

    [Header("Braking and Drifting Attributes")]
    [SerializeField] float breakTorgue = 10000f;

    [SerializeField] KartInputReader input;

    Rigidbody rb;

    float brakeVelocity;

    private void Start() {
      rb = GetComponent<Rigidbody>();
      input.Enable();

      foreach (var axleInfo in axleInfos) {
        axleInfo.originalForwardFriction = axleInfo.leftWheel.forwardFriction;
        axleInfo.originalSidewaysFriction = axleInfo.leftWheel.sidewaysFriction;
      }
    }

    private void FixedUpdate() {
      float verticalInput = AdjustInput(input.Move.y);
      float horizontalInput = AdjustInput(input.Move.x);

      float motor = maxMotorTorque * verticalInput;
      float steering = maxSteeringAngle * horizontalInput;

      UpdateAxeles(motor, steering);
    }

    float AdjustInput(float input) {
      return input switch {
        >= .7f => 1f,
        <= -.7f => -1f,
        _ => input
      };
    }

    private void UpdateAxeles(float motor, float steering) {
      foreach (var axleInfo in axleInfos) {
        HandleSteering(axleInfo, steering);
        HandleMotor(axleInfo, motor);
        HandleBrakesAndDrift(axleInfo);
        UpdateWheelVisuals(axleInfo.leftWheel);
        UpdateWheelVisuals(axleInfo.rightWheel);
      }
    }

    private void HandleSteering(AxleInfo axleInfo, float steering) {
      if (axleInfo.steering) {
        axleInfo.leftWheel.steerAngle = Mathf.Lerp(axleInfo.leftWheel.steerAngle, steering, 10 * Time.fixedDeltaTime);
        axleInfo.rightWheel.steerAngle = Mathf.Lerp(axleInfo.rightWheel.steerAngle, steering, 10 * Time.fixedDeltaTime);
      }
    }

    private void HandleMotor(AxleInfo axleInfo, float motor) {
      if (axleInfo.motor) {
        axleInfo.leftWheel.motorTorque = motor;
        axleInfo.rightWheel.motorTorque = motor;
      }
    }

    private void HandleBrakesAndDrift(AxleInfo axleInfo) {
      if (axleInfo.motor) {
        if (input.IsBraking) {
          rb.constraints = RigidbodyConstraints.FreezeRotationX;

          float newZ = Mathf.SmoothDamp(rb.velocity.z, 0, ref brakeVelocity, 1f);
          //rb.velocity = rb.velocity.With(z: newZ);
          rb.velocity.Set(0, newZ, 0);

          axleInfo.leftWheel.brakeTorque = breakTorgue;
          axleInfo.rightWheel.brakeTorque = breakTorgue;
        } else {
          rb.constraints = RigidbodyConstraints.None;

          axleInfo.leftWheel.brakeTorque = 0;
          axleInfo.rightWheel.brakeTorque = 0;
        }
      }
    }

    private void UpdateWheelVisuals(WheelCollider collider) {
      if (collider.transform.childCount == 0)
        return;

      Transform visualWheel = collider.transform.GetChild(0);

      Vector3 position;
      Quaternion rotation;
      collider.GetWorldPose(out position, out rotation);

      visualWheel.transform.position = position;
      visualWheel.transform.rotation = rotation;


    }

  }

}
