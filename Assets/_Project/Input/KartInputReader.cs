using UnityEngine;
using UnityEngine.InputSystem;
using static KartInputActions;

namespace Kart {

  [CreateAssetMenu(menuName = "Kart/Input Reader", fileName = "InputReader")]
  public class KartInputReader : ScriptableObject, IPlayerActions {

    public Vector2 Move => inputActions.Player.Move.ReadValue<Vector2>();
    public bool IsBraking => inputActions.Player.Brake.ReadValue<float>() > 0;

    KartInputActions inputActions;

    private void OnEnable() {
      if (inputActions == null) {
        inputActions = new KartInputActions();
        inputActions.Player.SetCallbacks(this);
      }
    }

    public void Enable() {
      inputActions.Enable();
    }

    public void OnMove(InputAction.CallbackContext context) {
    }

    public void OnLook(InputAction.CallbackContext context) {
    }

    public void OnFire(InputAction.CallbackContext context) {
    }

    public void OnBrake(InputAction.CallbackContext context) {
      Debug.Log("OnBrake");
    }
  }

}
