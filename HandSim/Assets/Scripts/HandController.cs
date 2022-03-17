using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class HandController : MonoBehaviour
{
    bool up = true;
    GameObject wrist;
    GameObject[] fingers = new GameObject[5];
    // Start is called before the first frame update
    void Start()
    {
        wrist = GameObject.Find("/KnobbyHand/Arm/Wrist");
        fingers[0] = GameObject.Find("Thumb/MetacarpophalangealJoint");
        fingers[1] = GameObject.Find("Index/MetacarpophalangealJoint");
        fingers[2] = GameObject.Find("Middle/MetacarpophalangealJoint");
        fingers[3] = GameObject.Find("Ring/MetacarpophalangealJoint");
        fingers[4] = GameObject.Find("Pinky/MetacarpophalangealJoint");

        
    }

    // Update is called once per frame
    void FixedUpdate()
    {
        if(up)
        {
            transform.Rotate(Vector3.forward * 60 * Time.deltaTime);
            foreach(GameObject finger in fingers)
            {
                finger.transform.Rotate(Vector3.right * 90 * Time.deltaTime);
            }
        }
        else
        {
            transform.Rotate(Vector3.back * 60 * Time.deltaTime);
            foreach(GameObject finger in fingers)
            {
                finger.transform.Rotate(Vector3.left * 90 * Time.deltaTime);
            }
        }

        if(transform.eulerAngles.z >= 90 || transform.eulerAngles.z <= 0)
        {
            up = !up;
        }
    }
}
