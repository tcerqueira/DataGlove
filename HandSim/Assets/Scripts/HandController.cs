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

        // Receive a message and write it to the console.
        IPEndPoint e = new IPEndPoint(IPAddress.Any, 5433);
        UdpClient u = new UdpClient(e);

        UdpState s = new UdpState();
        s.e = e;
        s.u = u;

        Debug.Log("listening for messages");
        u.BeginReceive(new AsyncCallback(OnReceive), s);
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
