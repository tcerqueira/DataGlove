using System.Collections;
using System.Collections.Generic;
using UnityEngine;

using System.Net.Sockets;
using System.Net;
using System.Text;
using System;

public class HandController : MonoBehaviour
{
    public struct UdpState
    {
        public UdpClient u;
        public IPEndPoint e;
    }

    public struct Finger
    {
        public Transform[] joints;
    }

    Transform wrist;
    Finger[] fingers = new Finger[5];

    // Start is called before the first frame update
    void Start()
    {
        wrist = GameObject.Find("/KnobbyHand/Arm/Wrist").transform;
        Transform[] fingerRoots = new Transform[] {
            GameObject.Find("Thumb").transform,
            GameObject.Find("Index").transform,
            GameObject.Find("Middle").transform,
            GameObject.Find("Ring").transform,
            GameObject.Find("Pinky").transform,
        };

        for(int i = 0; i < 5; i++)
        {
            fingers[i].joints = new Transform[3];
            fingers[i].joints[0] = fingerRoots[i].Find("MetacarpophalangealJoint");
            fingers[i].joints[1] = fingers[i].joints[0].Find("Proximal/ProximalinterphalangealJoint");
            fingers[i].joints[2] = fingers[i].joints[1].Find("Intermediate/DistalinterphalangealJoint");
            if(!fingers[i].joints[0] || !fingers[i].joints[1] || !fingers[i].joints[2]) {
                Debug.Log($"Error finding joints: Finger {i}");
            }
        }

        // Receive a message and write it to the console.
        IPEndPoint e = new IPEndPoint(IPAddress.Any, 5433);
        UdpClient u = new UdpClient(e);

        UdpState s = new UdpState();
        s.e = e;
        s.u = u;

        u.BeginReceive(new AsyncCallback(OnReceive), s);
        Debug.Log("Listening...");
    }

    // Update is called once per frame
    void FixedUpdate()
    {
        // if(up)
        // {
        //     transform.Rotate(Vector3.forward * 60 * Time.deltaTime);
        //     foreach(GameObject finger in fingers)
        //     {
        //         finger.transform.Rotate(Vector3.right * 90 * Time.deltaTime);
        //     }
        // }
        // else
        // {
        //     transform.Rotate(Vector3.back * 60 * Time.deltaTime);
        //     foreach(GameObject finger in fingers)
        //     {
        //         finger.transform.Rotate(Vector3.left * 90 * Time.deltaTime);
        //     }
        // }

        // if(transform.eulerAngles.z >= 90 || transform.eulerAngles.z <= 0)
        // {
        //     up = !up;
        // }
    }

    private void OnReceive(IAsyncResult ar)
    {
        UdpState state = (UdpState)(ar.AsyncState);
        UdpClient client = state.u;
        IPEndPoint endpoint = state.e;

        byte[] receiveBytes = client.EndReceive(ar, ref endpoint);
        string receiveString = Encoding.ASCII.GetString(receiveBytes);

        Debug.Log($"Received: {receiveString}");
        client.BeginReceive(new AsyncCallback(OnReceive), state);
    }
}
