/*************************************************************************
 *                                                                       *
 * Open Dynamics Engine, Copyright (C) 2001,2002 Russell L. Smith.       *
 * All rights reserved.  Email: russ@q12.org   Web: www.q12.org          *
 *                                                                       *
 * This library is free software; you can redistribute it and/or         *
 * modify it under the terms of EITHER:                                  *
 *   (1) The GNU Lesser General Public License as published by the Free  *
 *       Software Foundation; either version 2.1 of the License, or (at  *
 *       your option) any later version. The text of the GNU Lesser      *
 *       General Public License is included with this library in the     *
 *       file LICENSE.TXT.                                               *
 *   (2) The BSD-style license that is included with this library in     *
 *       the file LICENSE-BSD.TXT.                                       *
 *                                                                       *
 * This library is distributed in the hope that it will be useful,       *
 * but WITHOUT ANY WARRANTY; without even the implied warranty of        *
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the files    *
 * LICENSE.TXT and LICENSE-BSD.TXT for more details.                     *
 *                                                                       *
 *************************************************************************/

#include <ode/ode.h>
#include <drawstuff/drawstuff.h>
#include "texturepath.h"
#include "torrus_geom.h"
#include "basket_geom.h" // this is our world mesh
#include <iostream>

 static dGeomID sbody_mesh;

 using namespace std;

#ifdef _MSC_VER
#pragma warning(disable:4244 4305)  // for VC++, no precision loss complaints
#endif


#define	DEGTORAD			0.01745329251994329577f				//!< PI / 180.0, convert degrees to radians

int g_allow_trimesh;

// Our heightfield geom
dGeomID gheight;



// Heightfield dimensions

#define HFIELD_WSTEP			15			// Vertex count along edge >= 2
#define HFIELD_DSTEP			31

#define HFIELD_WIDTH			REAL( 4.0 )
#define HFIELD_DEPTH			REAL( 8.0 )

#define HFIELD_WSAMP			( HFIELD_WIDTH / ( HFIELD_WSTEP-1 ) )
#define HFIELD_DSAMP			( HFIELD_DEPTH / ( HFIELD_DSTEP-1 ) )




// select correct drawing functions

#ifdef dDOUBLE
#define dsDrawBox dsDrawBoxD
#define dsDrawSphere dsDrawSphereD
#define dsDrawCylinder dsDrawCylinderD
#define dsDrawCapsule dsDrawCapsuleD
#define dsDrawConvex dsDrawConvexD
#define dsDrawTriangle dsDrawTriangleD
#endif


// some constants

#define NUM 500			// max number of objects
#define DENSITY (1.0)		// density of all objects
#define GPB 3  		// maximum number of geometries per body
#define MAX_CONTACTS 128		// maximum number of contact points per body
#define WORLD_STEP 0.01


// dynamics and collision objects

struct MyObject {
  dBodyID body;			// the body
  dGeomID geom[GPB];		// geometries representing this body

  // Trimesh only - double buffered matrices for 'last transform' setup
  dReal matrix_dblbuff[ 16 * 2 ];
  int last_matrix_index;
};

static int num=0;		// number of objects in simulation
static int nextobj=0;		// next object to recycle if num==NUM
static dWorldID world;
static dSpaceID space;
static MyObject obj[NUM];
static dJointGroupID contactgroup;
static int selected = -1;	// selected object
static int show_aabb = 0;	// show geom AABBs?
static int show_contacts = 1;	// show contact points?
static int random_pos = 0;	// drop objects from random position?
static int write_world = 0;
static int collisions = 1;
static double time_pass = 0;

static dGeomID world_floor;

static bool create_mesh = false;




//============================

dGeomID TriMesh1;
dGeomID TriMesh2;
//static dTriMeshDataID TriData1, TriData2;  // reusable static trimesh data

//============================




// this is called by dSpaceCollide when two objects in space are
// potentially colliding.

static void nearCallback (void *data, dGeomID o1, dGeomID o2)
{
  int i;
  // if (o1->body && o2->body) return;

  // exit without doing anything if the two bodies are connected by a joint
  dBodyID b1 = dGeomGetBody(o1);
  dBodyID b2 = dGeomGetBody(o2);
  if (b1 && b2 && dAreConnectedExcluding (b1,b2,dJointTypeContact)) return;

  dContact contact[MAX_CONTACTS];   // up to MAX_CONTACTS contacts per box-box
  for (i=0; i<MAX_CONTACTS; i++) {
    contact[i].surface.mode = dContactBounce | dContactSoftCFM;
    contact[i].surface.mu = 0.1;  //ring friction
    contact[i].surface.mu2 = 0;
    contact[i].surface.bounce = 0.1;
    contact[i].surface.bounce_vel = 0.1;
    contact[i].surface.soft_cfm = 0.01;
  }
  if (int numc = dCollide (o1,o2,MAX_CONTACTS,&contact[0].geom,
			   sizeof(dContact))) {
    dMatrix3 RI;
    dRSetIdentity (RI);
    const dReal ss[3] = {0.02,0.02,0.02};
    for (i=0; i<numc; i++) {
      dJointID c = dJointCreateContact (world,contactgroup,contact+i);
      if (collisions || ( o1 == world_floor || o2 == world_floor))
      {
      	dJointAttach (c,b1,b2);
      }
      	
      if (show_contacts) dsDrawBox (contact[i].geom.pos,RI,ss);
    }
  }
}


// start simulation - set viewpoint

static void start()
{
  dAllocateODEDataForThread(dAllocateMaskAll);

  static float xyz[3] = {2.1640f,-1.3079f,1.7600f};
  static float hpr[3] = {125.5000f,-17.0000f,0.0000f};
  dsSetViewpoint (xyz,hpr);
  printf ("To drop another object, press:\n");
  printf ("   m for a trimesh.\n");
  printf ("To select an object, press space.\n");
  printf ("To disable the selected object, press d.\n");
  printf ("To enable the selected object, press e.\n");
  printf ("To toggle showing the geom AABBs, press a.\n");
  printf ("To toggle showing the contact points, press t.\n");
  printf ("To toggle dropping from random position/orientation, press r.\n");
  printf ("To save the current state to 'state.dif', press g.\n");
  printf ("Press n to disable collisions.\n");
}


char locase (char c)
{
  if (c >= 'A' && c <= 'Z') return c - ('a'-'A');
  else return c;
}


void create_torus ( dReal pos_x, dReal pos_y, dReal pos_z, dReal rot_x, dReal rot_y, dReal rot_z, dReal deg)
{

	size_t i;
	int j,k;
	dReal sides[3];
	dMass m;

	if ( num < NUM )
	{
		i = num;
		num++;
	}
	else
	{
		i = nextobj;
		nextobj++;
		if (nextobj >= num) nextobj = 0;

		// destroy the body and geoms for slot i
		dBodyDestroy (obj[i].body);
		for (k=0; k < GPB; k++)
		{
			if (obj[i].geom[k]) dGeomDestroy (obj[i].geom[k]);
		}
		memset (&obj[i],0,sizeof(obj[i]));
	}

	obj[i].body = dBodyCreate (world);
	for (k=0; k<3; k++) sides[k] = dRandReal()*0.5+0.1;

	dReal maxheight = 10;
	for (k=0; k<num; k++) {
		const dReal *pos = dBodyGetPosition (obj[k].body);
		if (pos[2] > maxheight) maxheight = pos[2];
	}

	//Set position =====================
	dBodySetPosition (obj[i].body, pos_x, pos_y, pos_z);
	
	dBodySetData (obj[i].body,(void*) i);

	dTriMeshDataID new_tmdata = dGeomTriMeshDataCreate();
	dGeomTriMeshDataBuildSingle(new_tmdata, &Vertices[0], 3 * sizeof(float), VertexCount, 
		&Indices[0], IndexCount, 3 * sizeof(dTriIndex));

	obj[i].geom[0] = dCreateTriMesh(space, new_tmdata, 0, 0, 0);

	dMassSetTrimesh( &m, DENSITY, obj[i].geom[0] );
	printf("mass at %f %f %f\n", m.c[0], m.c[1], m.c[2]);
	dGeomSetPosition(obj[i].geom[0], -m.c[0], -m.c[1], -m.c[2]);
	dMassTranslate(&m, -m.c[0], -m.c[1], -m.c[2]);


	//set rotation matrix  ====================
	dMatrix3 R;
	dRSetIdentity (R);

	dRFromAxisAndAngle (R, rot_x, rot_y, rot_z, deg * DEGTORAD);

	dBodySetRotation (obj[i].body,R);

	for (k=0; k < GPB; k++)
	{
		if (obj[i].geom[k]) dGeomSetBody (obj[i].geom[k],obj[i].body);
	}

	dBodySetMass (obj[i].body,&m);
}

// called when a key pressed

static void command (int cmd)
{
	size_t i;
	int j,k;
	dReal sides[3];
	dMass m;

	cmd = locase (cmd);

	

	//
	// Geom Creation
	//

	

	if (cmd == 'n')
		{
			collisions = !collisions;
		}

	if (cmd == 'g')
	{
		write_world = 1;
	}

	if ( cmd == 'm' && g_allow_trimesh )
	{

		create_mesh = true;
	}


	//
	// Control Commands
	//

	if (cmd == ' ') {
		selected++;
		if (selected >= num) selected = 0;
		if (selected < 0) selected = 0;
	}
	else if (cmd == 'd' && selected >= 0 && selected < num) {
		dBodyDisable (obj[selected].body);
	}
	else if (cmd == 'e' && selected >= 0 && selected < num) {
		dBodyEnable (obj[selected].body);
	}
	else if (cmd == 'a') {
		show_aabb ^= 1;
	}
	else if (cmd == 't') {
		show_contacts ^= 1;
	}
	else if (cmd == 'r') {
		random_pos ^= 1;
	}
	else if (cmd == '1') {
		write_world = 1;
	}
}

// simulation loop

static void simLoop (int pause)
{
  int i,j;
  
  dsSetColor (0,0,2);
  

  dSpaceCollide (space,0,&nearCallback);

  
  //  Step in the animation

  //if (!pause) dWorldStep (world,0.05);
  for (int i = 0; i < 5; i++)
  {
  	if (!pause) dWorldQuickStep (world, WORLD_STEP);
  }
  
  time_pass += WORLD_STEP;


  if (write_world) {
    FILE *f = fopen ("state.dif","wt");
    if (f) {
      dWorldExportDIF (world,f,"X");
      fclose (f);
    }
    write_world = 0;
  }

 

 	//Start create mesh

	if ( create_mesh)
	{
	  	int width = 10;  //short way
		int length = 30;  //long way

		
		dReal curr_length = -17;  


		
		for (int k = 0; k < length; k++)
		{

			dReal curr_width = -30;  

			for (int l = 0; l < width; l++)
			{
				//flip every other ring.
				if (k%2)
				{
					create_torus(curr_width + 1.2, curr_length, 20, 1, 0, 0, 70);
				}
				else
				{
					create_torus(curr_width, curr_length, 20, 1, 0, 0, 110);
				}
				
				curr_width += 2.4;
			} 
			curr_length += 0.9;
		}
		create_mesh = false;
	}


  	//end create mesh


	// remove all contact joints
	dJointGroupEmpty (contactgroup);


	dsSetColor (1,1,0);
	dsSetTexture (DS_WOOD);
	for (i=0; i<num; i++)
	{
		for (j=0; j < GPB; j++)
		{
			if (i==selected)
			{
				dsSetColor (0,0.7,1);
			}
			else if (! dBodyIsEnabled (obj[i].body))
			{
				dsSetColor (1,0.8,0);
			}
			else 
			{
				dsSetColor (1,1,0);
			}

			if ( obj[i].geom[j] && dGeomGetClass(obj[i].geom[j]) == dTriMeshClass )
			{
				dTriIndex* Indices = (dTriIndex*)::Indices;

				// assume all trimeshes are drawn as bunnies
				const dReal* Pos = dGeomGetPosition(obj[i].geom[j]);
				const dReal* Rot = dGeomGetRotation(obj[i].geom[j]);

				for (int ii = 0; ii < IndexCount / 3; ii++)
				{
					const dReal v[9] = { // explicit conversion from float to dReal
						Vertices[Indices[ii * 3 + 0] * 3 + 0],
							Vertices[Indices[ii * 3 + 0] * 3 + 1],
							Vertices[Indices[ii * 3 + 0] * 3 + 2],
							Vertices[Indices[ii * 3 + 1] * 3 + 0],
							Vertices[Indices[ii * 3 + 1] * 3 + 1],
							Vertices[Indices[ii * 3 + 1] * 3 + 2],
							Vertices[Indices[ii * 3 + 2] * 3 + 0],
							Vertices[Indices[ii * 3 + 2] * 3 + 1],
							Vertices[Indices[ii * 3 + 2] * 3 + 2]
					};
					dsDrawTriangle(Pos, Rot, &v[0], &v[3], &v[6], 1);
				}

				// tell the tri-tri collider the current transform of the trimesh --
				// this is fairly important for good results.

				// Fill in the (4x4) matrix.
				dReal* p_matrix = obj[i].matrix_dblbuff + ( obj[i].last_matrix_index * 16 );

				p_matrix[ 0 ] = Rot[ 0 ];	p_matrix[ 1 ] = Rot[ 1 ];	p_matrix[ 2 ] = Rot[ 2 ];	p_matrix[ 3 ] = 0;
				p_matrix[ 4 ] = Rot[ 4 ];	p_matrix[ 5 ] = Rot[ 5 ];	p_matrix[ 6 ] = Rot[ 6 ];	p_matrix[ 7 ] = 0;
				p_matrix[ 8 ] = Rot[ 8 ];	p_matrix[ 9 ] = Rot[ 9 ];	p_matrix[10 ] = Rot[10 ];	p_matrix[11 ] = 0;
				p_matrix[12 ] = Pos[ 0 ];	p_matrix[13 ] = Pos[ 1 ];	p_matrix[14 ] = Pos[ 2 ];	p_matrix[15 ] = 1;

				// Flip to other matrix.
				obj[i].last_matrix_index = !obj[i].last_matrix_index;

				// Apply the 'other' matrix which is the oldest.
				dGeomTriMeshSetLastTransform( obj[i].geom[j], 
					*(dMatrix4*)( obj[i].matrix_dblbuff + ( obj[i].last_matrix_index * 16 ) ) );
			}
			else
			{
				//drawGeom (obj[i].geom[j],0,0,show_aabb);
			}
		}
	}

	if ( show_aabb )
	{
		// draw the bounding box for this geom
		dReal aabb[6];
		dGeomGetAABB (gheight,aabb);
		dVector3 bbpos;
		for (i=0; i<3; i++) bbpos[i] = 0.5*(aabb[i*2] + aabb[i*2+1]);
		dVector3 bbsides;
		for (i=0; i<3; i++) bbsides[i] = aabb[i*2+1] - aabb[i*2];
		dMatrix3 RI;
		dRSetIdentity (RI);
		dsSetColorAlpha (1,0,0,0.5);
		dsDrawBox (bbpos,RI,bbsides);
	}


	// draw static body trimesh ===================================================
	dsSetColor(0.4,0.7,0.9);
	dsSetTexture (DS_NONE);

	const dReal* Pos = dGeomGetPosition(sbody_mesh);
	//dIASSERT(dVALIDVEC3(Pos));
	float pos[3] = { Pos[0], Pos[1], Pos[2] };

	const dReal* Rot = dGeomGetRotation(sbody_mesh);
	//dIASSERT(dVALIDMAT3(Rot));
	float rot[12] = { Rot[0], Rot[1], Rot[2], Rot[3], Rot[4], Rot[5], Rot[6], Rot[7], Rot[8], Rot[9], Rot[10], Rot[11] };

	int numi = sizeof(world_indices)  / sizeof(dTriIndex);

	for (int i=0; i<numi/3; i++)
	{
	int i0 = world_indices[i*3+0];
	int i1 = world_indices[i*3+1];
	int i2 = world_indices[i*3+2];
	float *v0 = world_vertices+i0*3;
	float *v1 = world_vertices+i1*3;
	float *v2 = world_vertices+i2*3;
	dsDrawTriangle(pos, rot, v0,v1,v2, true); // single precision draw
	}
	//   end static body trimesh ========================================================

}


int main (int argc, char **argv)
{
        printf("ODE configuration: %s\n", dGetConfiguration());
        
	// Is trimesh support built into this ODE?
	g_allow_trimesh = dCheckConfiguration( "ODE_EXT_trimesh" );

	// setup pointers to drawstuff callback functions
	dsFunctions fn;
	fn.version = DS_VERSION;
	fn.start = &start;
	fn.step = &simLoop;
	fn.command = &command;
	fn.stop = 0;
	fn.path_to_textures = DRAWSTUFF_TEXTURE_PATH;

	// create world
	dInitODE2(0);
	world = dWorldCreate();
	space = dHashSpaceCreate (0);
	contactgroup = dJointGroupCreate (0);
	//dWorldSetGravity (world,0,0,-5);   //world gravity
	dWorldSetGravity (world,0,0,0); 
	dWorldSetCFM (world,1e-5);
	dWorldSetAutoDisableFlag (world,1);
	dWorldSetContactMaxCorrectingVel (world,0.1);
	dWorldSetContactSurfaceLayer (world,0.001);
	memset (obj,0,sizeof(obj));


	//   Draw world static body geometry
	dMatrix3 R;
	 // Create a static world using a triangle mesh that we can collide with.
	int numv = sizeof(world_vertices)/(3*sizeof(float));
	int numi = sizeof(world_indices)/ sizeof(dTriIndex);
	printf("numv=%d, numi=%d\n", numv, numi);
	dTriMeshDataID Data = dGeomTriMeshDataCreate();

	//  fprintf(stderr,"Building Single Precision Mesh\n");

	dGeomTriMeshDataBuildSingle
	(
		Data, 
		world_vertices, 
		3 * sizeof(float), 
		numv, 
		world_indices, 
		numi, 
		3 * sizeof(dTriIndex)
	);

	sbody_mesh = dCreateTriMesh(space, Data, 0, 0, 0);
	dGeomTriMeshEnableTC(sbody_mesh, dSphereClass, false);
	dGeomTriMeshEnableTC(sbody_mesh, dBoxClass, false);
	dGeomSetPosition(sbody_mesh, 0, 0, 0.5);
	dRSetIdentity(R);
	//dIASSERT(dVALIDMAT3(R));

	dGeomSetRotation (sbody_mesh, R);
	//end static body geom

	//dBodySetData (dBodyID, void *data)

#if 1

  dWorldSetAutoDisableAverageSamplesCount( world, 1 );

#endif

	// base plane to catch overspill
  world_floor = dCreatePlane( space, 0, 0, 1, 0 );

  dThreadingImplementationID threading = dThreadingAllocateMultiThreadedImplementation();
  dThreadingThreadPoolID pool = dThreadingAllocateThreadPool(8, 0, dAllocateFlagBasicData, NULL);
  dThreadingThreadPoolServeMultiThreadedImplementation(pool, threading);
  // dWorldSetStepIslandsProcessingMaxThreadCount(world, 1);
  dWorldSetStepThreadingImplementation(world, dThreadingImplementationGetFunctions(threading), threading);

	// run simulation
	dsSimulationLoop (argc,argv,352,288,&fn);


  dThreadingImplementationShutdownProcessing(threading);
  dThreadingFreeThreadPool(pool);
  dWorldSetStepThreadingImplementation(world, NULL, NULL);
  dThreadingFreeImplementation(threading);

	dJointGroupDestroy (contactgroup);
	dSpaceDestroy (space);
	dWorldDestroy (world);

	// destroy heightfield data, because _we_ own it not ODE
	//dGeomHeightfieldDataDestroy( heightid );

	dCloseODE();
	return 0;
}
