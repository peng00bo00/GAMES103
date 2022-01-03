using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using System;
using System.IO;

public class FVM : MonoBehaviour
{
	float dt 			= 0.002f;
    float mass 			= 1;
	float stiffness_0	= 10000.0f;
    float stiffness_1 	= 5000.0f;
    float damp			= 0.999f;

	Vector3 g = new Vector3(0, -9.8f, 0);
	float floor = -3.0f;

	float muN = 1.0f;
	float muT = 1.0f;
	int[] 		Tet;
	int tet_number;			//The number of tetrahedra

	Vector3[] 	Force;
	Vector3[] 	V;
	Vector3[] 	X;
	int number;				//The number of vertices

	Matrix4x4[] inv_Dm;

	//For Laplacian smoothing.
	Vector3[]   V_sum;
	int[]		V_num;

	SVD svd = new SVD();

    // Start is called before the first frame update
    void Start()
    {
    	// FILO IO: Read the house model from files.
    	// The model is from Jonathan Schewchuk's Stellar lib.
    	{
    		string fileContent = File.ReadAllText("Assets/house2.ele");
    		string[] Strings = fileContent.Split(new char[]{' ', '\t', '\r', '\n'}, StringSplitOptions.RemoveEmptyEntries);
    		
    		tet_number=int.Parse(Strings[0]);
        	Tet = new int[tet_number*4];

    		for(int tet=0; tet<tet_number; tet++)
    		{
				Tet[tet*4+0]=int.Parse(Strings[tet*5+4])-1;
				Tet[tet*4+1]=int.Parse(Strings[tet*5+5])-1;
				Tet[tet*4+2]=int.Parse(Strings[tet*5+6])-1;
				Tet[tet*4+3]=int.Parse(Strings[tet*5+7])-1;
			}
    	}
    	{
			string fileContent = File.ReadAllText("Assets/house2.node");
    		string[] Strings = fileContent.Split(new char[]{' ', '\t', '\r', '\n'}, StringSplitOptions.RemoveEmptyEntries);
    		number = int.Parse(Strings[0]);
    		X = new Vector3[number];
       		for(int i=0; i<number; i++)
       		{
       			X[i].x=float.Parse(Strings[i*5+5])*0.4f;
       			X[i].y=float.Parse(Strings[i*5+6])*0.4f;
       			X[i].z=float.Parse(Strings[i*5+7])*0.4f;
       		}
    		//Centralize the model.
	    	Vector3 center=Vector3.zero;
	    	for(int i=0; i<number; i++)		center+=X[i];
	    	center=center/number;
	    	for(int i=0; i<number; i++)
	    	{
	    		X[i]-=center;
	    		float temp=X[i].y;
	    		X[i].y=X[i].z;
	    		X[i].z=temp;
	    	}
		}

        // tet_number=1;
        // Tet = new int[tet_number*4];
        // Tet[0]=0;
        // Tet[1]=1;
        // Tet[2]=2;
        // Tet[3]=3;

        // number=4;
        // X = new Vector3[number];
        // V = new Vector3[number];
        // Force = new Vector3[number];
        // X[0]= new Vector3(0, 0, 0);
        // X[1]= new Vector3(1, 0, 0);
        // X[2]= new Vector3(0, 1, 0);
        // X[3]= new Vector3(0, 0, 1);


        //Create triangle mesh.
       	Vector3[] vertices = new Vector3[tet_number*12];
        int vertex_number=0;
        for(int tet=0; tet<tet_number; tet++)
        {
        	vertices[vertex_number++]=X[Tet[tet*4+0]];
        	vertices[vertex_number++]=X[Tet[tet*4+2]];
        	vertices[vertex_number++]=X[Tet[tet*4+1]];

        	vertices[vertex_number++]=X[Tet[tet*4+0]];
        	vertices[vertex_number++]=X[Tet[tet*4+3]];
        	vertices[vertex_number++]=X[Tet[tet*4+2]];

        	vertices[vertex_number++]=X[Tet[tet*4+0]];
        	vertices[vertex_number++]=X[Tet[tet*4+1]];
        	vertices[vertex_number++]=X[Tet[tet*4+3]];

        	vertices[vertex_number++]=X[Tet[tet*4+1]];
        	vertices[vertex_number++]=X[Tet[tet*4+2]];
        	vertices[vertex_number++]=X[Tet[tet*4+3]];
        }

        int[] triangles = new int[tet_number*12];
        for(int t=0; t<tet_number*4; t++)
        {
        	triangles[t*3+0]=t*3+0;
        	triangles[t*3+1]=t*3+1;
        	triangles[t*3+2]=t*3+2;
        }
        Mesh mesh = GetComponent<MeshFilter> ().mesh;
		mesh.vertices  = vertices;
		mesh.triangles = triangles;
		mesh.RecalculateNormals ();


		V 	  = new Vector3[number];
        Force = new Vector3[number];
        V_sum = new Vector3[number];
        V_num = new int[number];

		//TODO: Need to allocate and assign inv_Dm
		inv_Dm = new Matrix4x4[tet_number];
		for (int tet = 0; tet < tet_number; tet++)
		{
			inv_Dm[tet] = Build_Edge_Matrix(tet);
			
		}
    }

    Matrix4x4 Build_Edge_Matrix(int tet)
    {
    	Matrix4x4 ret=Matrix4x4.zero;
    	//TODO: Need to build edge matrix here.

		Vector3 X0 = X[Tet[tet*4+0]];
		Vector3 X1 = X[Tet[tet*4+1]];
		Vector3 X2 = X[Tet[tet*4+2]];
		Vector3 X3 = X[Tet[tet*4+3]];

		ret.SetColumn(0, new Vector4(X1.x - X0.x, X1.y - X0.y, X1.z - X0.z, 0.0f));
		ret.SetColumn(1, new Vector4(X2.x - X0.x, X2.y - X0.y, X2.z - X0.z, 0.0f));
		ret.SetColumn(2, new Vector4(X3.x - X0.x, X3.y - X0.y, X3.z - X0.z, 0.0f));
		ret[3, 3] = 1.0f;

		// Debug.Log(ret.inverse);

		return ret.inverse;
    }


    void _Update()
    {
    	// Jump up.
		if(Input.GetKeyDown(KeyCode.Space))
    	{
    		for(int i=0; i<number; i++)
    			V[i].y+=0.2f;
    	}

    	for(int i=0 ;i<number; i++)
    	{
    		//TODO: Add gravity to Force.
			Force[i] = mass * g;
    	}

    	for(int tet=0; tet<tet_number; tet++)
    	{
    		//TODO: Deformation Gradient
			Vector3 X0 = X[Tet[tet*4+0]];
			Vector3 X1 = X[Tet[tet*4+1]];
			Vector3 X2 = X[Tet[tet*4+2]];
			Vector3 X3 = X[Tet[tet*4+3]];

			Matrix4x4 F = Matrix4x4.identity;
			F.SetColumn(0, new Vector4(X1.x - X0.x, X1.y - X0.y, X1.z - X0.z, 0.0f));
			F.SetColumn(1, new Vector4(X2.x - X0.x, X2.y - X0.y, X2.z - X0.z, 0.0f));
			F.SetColumn(2, new Vector4(X3.x - X0.x, X3.y - X0.y, X3.z - X0.z, 0.0f));

			F = F * inv_Dm[tet];
    		
    		// //TODO: Green Strain
			// Matrix4x4 G = F.transpose * F;

			// G[0, 0] -= 1;
			// G[1, 1] -= 1;
			// G[2, 2] -= 1;

			// G = scalarMultiply(G, 0.5f);
			// G[3, 3] = 1.0f;

    		// //TODO: Second PK Stress
			// Matrix4x4 S = Matrix4x4.zero;
			
			// S = scalarMultiply(G, 2*stiffness_1);
			// S = matrixAdd(S, scalarMultiply(Matrix4x4.identity, stiffness_0 * matrixTrace(G)));
			// S[3, 3] = 1.0f;

			// Matrix4x4 P = F * S;

			// SVD
			Matrix4x4 Usvd = Matrix4x4.identity;
			Matrix4x4 Ssvd = Matrix4x4.identity;
			Matrix4x4 Vsvd = Matrix4x4.identity;

			svd.svd(F, ref Usvd, ref Ssvd, ref Vsvd);

			float I  = Ssvd[0, 0]*Ssvd[0, 0] + Ssvd[1, 1]*Ssvd[1, 1] + Ssvd[2, 2]*Ssvd[2, 2];
			float II = Ssvd[0, 0]*Ssvd[0, 0]*Ssvd[0, 0]*Ssvd[0, 0]
			         + Ssvd[1, 1]*Ssvd[1, 1]*Ssvd[1, 1]*Ssvd[1, 1]
					 + Ssvd[2, 2]*Ssvd[2, 2]*Ssvd[2, 2]*Ssvd[2, 2];
			// float III= 0;

			float dWI = stiffness_0 * (I - 3) - stiffness_1 / 2;
			float dWII= stiffness_1 / 4;

			Matrix4x4 P = Matrix4x4.identity;
			P[0, 0] = 2 * Ssvd[0, 0] * dWI + 4 * Ssvd[0, 0] * Ssvd[0, 0] * Ssvd[0, 0] * dWII;
			P[1, 1] = 2 * Ssvd[1, 1] * dWI + 4 * Ssvd[1, 1] * Ssvd[1, 1] * Ssvd[1, 1] * dWII;
			P[2, 2] = 2 * Ssvd[2, 2] * dWI + 4 * Ssvd[2, 2] * Ssvd[2, 2] * Ssvd[2, 2] * dWII;

			P = Usvd * P * Vsvd.transpose;

    		//TODO: Elastic Force
			float det = inv_Dm[tet].determinant;
			Matrix4x4 f = P * inv_Dm[tet].transpose;
			f = scalarMultiply(f, -1.0f/(6*det));

			Vector3 f0 = Vector3.zero;
			f0.x = -(f[0, 0] + f[0, 1] + f[0, 2]);
			f0.y = -(f[1, 0] + f[1, 1] + f[1, 2]);
			f0.z = -(f[2, 0] + f[2, 1] + f[2, 2]);

			Force[Tet[tet*4+0]] += f0;
			Force[Tet[tet*4+1]] += new Vector3(f[0, 0], f[1, 0], f[2, 0]);
			Force[Tet[tet*4+2]] += new Vector3(f[0, 1], f[1, 1], f[2, 1]);
			Force[Tet[tet*4+3]] += new Vector3(f[0, 2], f[1, 2], f[2, 2]);
			
    	}

    	for(int i=0; i<number; i++)
    	{
    		//TODO: Update X and V here.
			V[i] += Force[i] / mass * dt;
			V[i] *= damp;

			X[i] += V[i] * dt;

    		//TODO: (Particle) collision with floor.
			if (X[i].y < floor) {
				X[i].y = floor;
				if (V[i].y < 0) {
					Vector3 VN = new Vector3(0, V[i].y, 0);
					Vector3 VT = V[i] - VN;

					VN = -muN * VN;
					float a = Math.Max(0.0f, 1.0f-muT*(1.0f + muN)*VN.magnitude/VT.magnitude);

					VT = a * VT;

					V[i] = VT + VN;
				}
			}
    	}

		// Laplacian smoothing
		for(int i=0; i<number; i++)
    	{
    		V_sum[i] = Vector3.zero;
			V_num[i] = 0;
    	}

		for (int tet=0; tet<tet_number; tet++)
		{
			Vector3 v = Vector3.zero;

			// take sum of the vertice velocity
			for (int i = 0; i < 4; i++)
			{
				v += V[Tet[tet*4+i]];
			}

			// add to each vertex
			for (int i = 0; i < 4; i++)
			{
				V_sum[Tet[tet*4+i]] += v;
				V_num[Tet[tet*4+i]] += 4;
			}
		}

		for(int i=0; i<number; i++)
    	{
    		V[i] = V_sum[i] / V_num[i];
    	}
    }

    // Update is called once per frame
    void Update()
    {
    	for(int l=0; l<10; l++)
    		 _Update();

    	// Dump the vertex array for rendering.
    	Vector3[] vertices = new Vector3[tet_number*12];
        int vertex_number=0;
        for(int tet=0; tet<tet_number; tet++)
        {
        	vertices[vertex_number++]=X[Tet[tet*4+0]];
        	vertices[vertex_number++]=X[Tet[tet*4+2]];
        	vertices[vertex_number++]=X[Tet[tet*4+1]];
        	vertices[vertex_number++]=X[Tet[tet*4+0]];
        	vertices[vertex_number++]=X[Tet[tet*4+3]];
        	vertices[vertex_number++]=X[Tet[tet*4+2]];
        	vertices[vertex_number++]=X[Tet[tet*4+0]];
        	vertices[vertex_number++]=X[Tet[tet*4+1]];
        	vertices[vertex_number++]=X[Tet[tet*4+3]];
        	vertices[vertex_number++]=X[Tet[tet*4+1]];
        	vertices[vertex_number++]=X[Tet[tet*4+2]];
        	vertices[vertex_number++]=X[Tet[tet*4+3]];
        }
        Mesh mesh = GetComponent<MeshFilter> ().mesh;
		mesh.vertices  = vertices;
		mesh.RecalculateNormals ();
    }

	Matrix4x4 scalarMultiply(Matrix4x4 m, float a) {
		Matrix4x4 ret = Matrix4x4.zero;

		for (int i = 0; i < 4; i++)
		{
			for (int j = 0; j < 4; j++)
			{
				ret[i, j] = m[i, j] * a;
			}
		}

		return ret;
	}

	Matrix4x4 matrixAdd(Matrix4x4 m1, Matrix4x4 m2) {
		Matrix4x4 ret = Matrix4x4.zero;

		for (int i = 0; i < 4; i++)
		{
			for (int j = 0; j < 4; j++)
			{
				ret[i, j] = m1[i, j] + m2[i, j];
			}
		}

		return ret;
	}

	float matrixTrace(Matrix4x4 m, int num=3) {
		float ret = 0.0f;

		for (int i = 0; i < num; i++)
		{
			ret += m[i, i];
		}

		return ret;
	}
}
