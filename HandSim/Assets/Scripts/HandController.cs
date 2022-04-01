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

    // Mutex for latest glove data
    readonly object gloveDataLock = new object();
    byte[] gloveData;

    Boolean up = true;
    public Transform wrist;
    public Finger[] fingers = new Finger[5];
    public Transform[] fingerRoots;

    // Start is called before the first frame update
    void Start()
    {
        for (int i = 0; i < 5; i++)
        {
            fingers[i].joints = new Transform[3];
            fingers[i].joints[0] = fingerRoots[i].Find("MetacarpophalangealJoint");
            fingers[i].joints[1] = fingers[i].joints[0].Find("Proximal/ProximalinterphalangealJoint");
            fingers[i].joints[2] = fingers[i].joints[1].Find("Intermediate/DistalinterphalangealJoint");
            if (!fingers[i].joints[0] || !fingers[i].joints[1] || !fingers[i].joints[2])
            {
                Debug.Log($"Error finding joints: Finger {i}");
            }
        }

        // Test JSON parsing
        // HandPose pose = new HandPose
        // {
        //     wrist = new SQuaternion(0, 0, 1, 1),
        //     fingers = new FingerPose[] {
        //         new FingerPose { joints = new SQuaternion[]{ new SQuaternion(1,0,0,1), new SQuaternion(0,0,0,0), new SQuaternion(0,0,0,0) }},
        //         new FingerPose { joints = new SQuaternion[]{ new SQuaternion(1,0,0,1), new SQuaternion(0,0,0,0), new SQuaternion(0,0,0,0) }},
        //         new FingerPose { joints = new SQuaternion[]{ new SQuaternion(1,0,0,1), new SQuaternion(0,0,0,0), new SQuaternion(0,0,0,0) }},
        //         new FingerPose { joints = new SQuaternion[]{ new SQuaternion(1,0,0,1), new SQuaternion(0,0,0,0), new SQuaternion(0,0,0,0) }},
        //         new FingerPose { joints = new SQuaternion[]{ new SQuaternion(1,0,0,1), new SQuaternion(0,0,0,0), new SQuaternion(0,0,0,0) }}
        //     }
        // };
        // String poseStr = JsonUtility.ToJson(pose);
        // HandPose pose2 = JsonUtility.FromJson<HandPose>(poseStr);
        // Debug.Log(pose2);

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
        if (up)
        {
            wrist.Rotate(Vector3.forward * 60 * Time.deltaTime);
            foreach (Finger finger in fingers)
            {
                finger.joints[0].Rotate(Vector3.right * 90 * Time.deltaTime);
            }
        }
        else
        {
            wrist.Rotate(Vector3.back * 60 * Time.deltaTime);
            foreach (Finger finger in fingers)
            {
                finger.joints[0].Rotate(Vector3.left * 90 * Time.deltaTime);
            }
        }

        if (wrist.eulerAngles.z >= 90 || wrist.eulerAngles.z <= 0)
        {
            up = !up;
        }
    }

    private void OnReceive(IAsyncResult ar)
    {
        UdpState state = (UdpState)(ar.AsyncState);
        UdpClient client = state.u;
        IPEndPoint endpoint = state.e;

        lock (gloveDataLock)
        {
            gloveData = client.EndReceive(ar, ref endpoint);
        }
        byte[] receiveBytes = (byte[])gloveData.Clone();
        string receiveString = Encoding.ASCII.GetString(receiveBytes);

        Debug.Log($"Received: {receiveString}");
        client.BeginReceive(new AsyncCallback(OnReceive), state);
    }

    private HandPose? ParseGloveData()
    {
        return null;
    }
}

[Serializable]
public struct FingerPose
{
    public SQuaternion[] joints;
}

[Serializable]
public struct HandPose
{
    public SQuaternion wrist;
    public FingerPose[] fingers;
}

// Serializable Quaternion
[Serializable]
public struct SQuaternion
{
    public float x;
    public float y;
    public float z;
    public float w;

    public SQuaternion(float rX, float rY, float rZ, float rW)
    {
        x = rX;
        y = rY;
        z = rZ;
        w = rW;
    }

    public override string ToString()
    {
        return String.Format("[{0}, {1}, {2}, {3}]", x, y, z, w);
    }

    public static implicit operator Quaternion(SQuaternion rValue)
    {
        return new Quaternion(rValue.x, rValue.y, rValue.z, rValue.w);
    }

    public static implicit operator SQuaternion(Quaternion rValue)
    {
        return new SQuaternion(rValue.x, rValue.y, rValue.z, rValue.w);
    }
}
