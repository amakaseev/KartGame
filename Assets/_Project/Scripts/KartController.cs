using System;
using System.Linq;
using UnityEngine;

namespace Kart {

  public class KartController : MonoBehaviour {
    [Header("Refs")]
    [SerializeField] KartInputReader input;

    Rigidbody rb;

    private void Start() {
      rb = GetComponent<Rigidbody>();
      input.Enable();
    }

    private void FixedUpdate() {
      float verticalInput = input.Move.y;
      float horizontalInput = input.Move.x;
    }
  }

}
