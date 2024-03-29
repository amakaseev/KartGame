using System;
using System.Linq;
using UnityEngine;
using Utilities;

namespace Kart {
  [Serializable]
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
    [SerializeField] AnimationCurve turnCurve;
    [SerializeField] float turnStrength = 1500f;

    [Header("Braking and Drifting")]
    [SerializeField] float driftSteerMultiplier = 1.5f; // Change in steering duration a drift
    [SerializeField] float breakTorgue = 10000f;

    [Header("Physics")]
    [SerializeField] Transform centerOfMass;
    [SerializeField] float downForce = 100f;
    [SerializeField] float gravity = Physics.gravity.y;
    [SerializeField] float lateralGScale = 10f; // Scaling factor for lateral G forces

    [Header("Banking")]
    [SerializeField] float maxBankAngle = 5f;
    [SerializeField] float bankSpeed = 2f;

    [Header("Refs")]
    [SerializeField] KartInputReader input;
    Rigidbody rb;

    Vector3 kartVelocity;
    float brakeVelocity;
    float driftVelocity;

    RaycastHit hit;

    const float thresholdSpeed = 10f;
    const float centerOfMassOffset = -0.5f;
    Vector3 originalCenterOfMass;

    public bool IsGrounded = true;
    public Vector3 Velocity => kartVelocity;
    public float MaxSpeed => maxSpeed;

    private void Start() {
      rb = GetComponent<Rigidbody>();
      input.Enable();

      rb.centerOfMass = centerOfMass.localPosition;
      originalCenterOfMass = centerOfMass.localPosition;


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
      UpdateBanking(horizontalInput);

      kartVelocity = transform.InverseTransformDirection(rb.velocity);

      if (IsGrounded) {
        HandleGroundedMovement(verticalInput, horizontalInput);
      } else {
        HandleAirborneMovement(verticalInput, horizontalInput);
      }
    }

    private void HandleGroundedMovement(float verticalInput, float horizontalInput) {
      // Turn logic
      if (Mathf.Abs(verticalInput) > 0.1f || Mathf.Abs(kartVelocity.z) > 1) {
        float turnMultiplayer = Mathf.Clamp01(turnCurve.Evaluate(kartVelocity.magnitude / maxSpeed));
        rb.AddTorque(Vector3.up * horizontalInput * Mathf.Sign(kartVelocity.z) * turnStrength * 100f * turnMultiplayer);
      }

      // Acceleration logic
      if (!input.IsBraking) {
        float targetSpeed = verticalInput * maxSpeed;
        Vector3 forwardWithoutY = transform.forward.With(y: 0).normalized;
        rb.velocity = Vector3.Lerp(rb.velocity, forwardWithoutY * targetSpeed, Time.deltaTime);
      }

      // Downforce - always push the kart down, using lateral Gs to scale the force if the kart is movind sideways fast
      float speedFactor = Mathf.Clamp01(rb.velocity.magnitude / maxSpeed);
      float lateralG = Mathf.Abs(Vector3.Dot(rb.velocity, transform.right));
      float downForceFactor = Mathf.Max(speedFactor, lateralG / lateralGScale);
      rb.AddForce(-transform.up * downForce * rb.mass * downForceFactor);

      // Shift Center of Mass
      float speed = rb.velocity.magnitude;
      Vector3 centerOfMassAdjustment = (speed > thresholdSpeed)
        ? new Vector3(0f, 0f, Mathf.Abs(verticalInput) > 0.1f ? Mathf.Sign(verticalInput) * centerOfMassOffset : 0f)
        : Vector3.zero;
      rb.centerOfMass = originalCenterOfMass + centerOfMassAdjustment;
    }

    private void UpdateBanking(float horizontalInput) {
      float targetBankAngle = horizontalInput * -maxBankAngle;
      Vector3 currentEuler = transform.localEulerAngles;
      currentEuler.z = Mathf.LerpAngle(currentEuler.z, targetBankAngle, Time.deltaTime * bankSpeed);
      transform.localEulerAngles = currentEuler;
    }

    private void HandleAirborneMovement(float verticalInput, float horizontalInput) {
      rb.velocity = Vector3.Lerp(rb.velocity, rb.velocity + Vector3.down * gravity, Time.deltaTime * gravity);
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
        axleInfo.leftWheel.steerAngle = steering;
        axleInfo.rightWheel.steerAngle = steering;
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
          rb.velocity = rb.velocity.With(z: newZ);

          axleInfo.leftWheel.brakeTorque = breakTorgue;
          axleInfo.rightWheel.brakeTorque = breakTorgue;
          ApplyDriftFriction(axleInfo.leftWheel);
          ApplyDriftFriction(axleInfo.rightWheel);
        } else {
          rb.constraints = RigidbodyConstraints.None;

          axleInfo.leftWheel.brakeTorque = 0;
          axleInfo.rightWheel.brakeTorque = 0;
          ResetDriftFriction(axleInfo.leftWheel);
          ResetDriftFriction(axleInfo.rightWheel);
        }
      }
    }

    private void ResetDriftFriction(WheelCollider wheel) {
      AxleInfo axleInfo = axleInfos.FirstOrDefault(axle => axle.leftWheel == wheel || axle.rightWheel == wheel);
      if (axleInfo == null) return;

      wheel.forwardFriction = axleInfo.originalForwardFriction;
      wheel.sidewaysFriction = axleInfo.originalSidewaysFriction;
    }

    private void ApplyDriftFriction(WheelCollider wheel) {
      if (wheel.GetGroundHit(out var hit)) {
        wheel.forwardFriction = UpdateFriction(wheel.forwardFriction);
        wheel.sidewaysFriction = UpdateFriction(wheel.sidewaysFriction);
        IsGrounded = true;
      }
    }

    WheelFrictionCurve UpdateFriction(WheelFrictionCurve friction) {
      friction.stiffness = input.IsBraking ? Mathf.SmoothDamp(friction.stiffness, .5f, ref driftVelocity, Time.deltaTime * 2) : 1f;
      return friction;
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

    float AdjustInput(float input) {
      return input switch {
        >= .7f => 1f,
        <= -.7f => -1f,
        _ => input
      };
    }

  }

}
