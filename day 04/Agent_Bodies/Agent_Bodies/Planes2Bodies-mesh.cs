using System;
using System.Collections;
using System.Collections.Generic;

using Rhino;
using Rhino.Geometry;

using Grasshopper;
using Grasshopper.Kernel;
using Grasshopper.Kernel.Data;
using Grasshopper.Kernel.Types;

// <Custom using> 
using System.Threading;
using System.Threading.Tasks;
using System.Collections.Concurrent;
using System.Drawing;
// </Custom using> 


/// <summary>
/// This class will be instantiated on demand by the Script component.
/// </summary>
public class Script_Instance2 : GH_ScriptInstance // Script_Instance2 - name changed to avoid conflicts while writing code
{
    #region Utility functions
    /// <summary>Print a String to the [Out] Parameter of the Script component.</summary>
    /// <param name="text">String to print.</param>
    private void Print(string text) { __out.Add(text); }
    /// <summary>Print a formatted String to the [Out] Parameter of the Script component.</summary>
    /// <param name="format">String format.</param>
    /// <param name="args">Formatting parameters.</param>
    private void Print(string format, params object[] args) { __out.Add(string.Format(format, args)); }
    /// <summary>Print useful information about an object instance to the [Out] Parameter of the Script component. </summary>
    /// <param name="obj">Object instance to parse.</param>
    private void Reflect(object obj) { __out.Add(GH_ScriptComponentUtilities.ReflectType_CS(obj)); }
    /// <summary>Print the signatures of all the overloads of a specific method to the [Out] Parameter of the Script component. </summary>
    /// <param name="obj">Object instance to parse.</param>
    private void Reflect(object obj, string method_name) { __out.Add(GH_ScriptComponentUtilities.ReflectType_CS(obj, method_name)); }
    #endregion

    #region Members
    /// <summary>Gets the current Rhino document.</summary>
    private RhinoDoc RhinoDocument;
    /// <summary>Gets the Grasshopper document that owns this script.</summary>
    private GH_Document GrasshopperDocument;
    /// <summary>Gets the Grasshopper script component that owns this script.</summary>
    private IGH_Component Component;
    /// <summary>
    /// Gets the current iteration count. The first call to RunScript() is associated with Iteration==0.
    /// Any subsequent call within the same solution will increment the Iteration count.
    /// </summary>
    private int Iteration;
    #endregion

    /// <summary>
    /// This procedure contains the user code. Input parameters are provided as regular arguments, 
    /// Output parameters as ref arguments. You don't have to assign output parameters, 
    /// they will have a default value.
    /// </summary>
    private void RunScript(bool reset, bool go, List<Plane> Pl, double sR, List<Polyline> body, ref object iter, ref object Bodies, ref object Planes, ref object TipPoints)
    {
        // <Custom code> 

        // . . . . . . . . . . . . . . . . . . . . . . . . return on null data
        if (Pl == null || body == null) return;

        // variables for data extraction
        DataTree<Polyline> outBodies = new DataTree<Polyline>();
        GH_Plane[] outPlanes = new GH_Plane[Pl.Count];
        List<GH_Point> tipPoints = new List<GH_Point>();

        // . . . . . . . . . . . . . . . . . . . . . . . . initialize system
        if (reset || ABS == null)
        {
            ABS = new AgentBodySimulation(Pl, body, sR);
            iterationsCount = 0;
            Status = "";
        }

        //Print("{0}", ABS.Agents[0].Neighbours.Count);
        //Print(Status);

        if (go)
        {
            // update parameters
            //ABS.searchRadius = sR;

            ABS.Update();
            iterationsCount++;
            Component.ExpireSolution(true);
        }


        // . . . . . . . . . . . . . . . . . . . . . . . . extract geometries and data

        // necessary only in case of parallelization
        //for (int i = 0; i < ABS.Agents.Length; i++)
        //    outBodies.EnsurePath(new GH_Path(i));

        for (int i = 0; i < ABS.Agents.Length; i++)
        {
            outBodies.AddRange(ABS.Agents[i].ExtractBody(), new GH_Path(i));
            outPlanes[i] = new GH_Plane(ABS.Agents[i].agentPlane);
            for (int j = 0; j < ABS.Agents[i].body.Tips.Count; j++)
                tipPoints.Add(new GH_Point(ABS.Agents[i].body.Tips[j].pos));
        }

        iter = iterationsCount;
        Bodies = outBodies;
        Planes = outPlanes;
        TipPoints = tipPoints;
        // </Custom code> 
    }

    // <Custom additional code> 

    // global variables
    public AgentBodySimulation ABS;
    public static string Status = ""; // for debugging
    public int iterationsCount;

    // .................................................................... classes 
    // .............................................................................
    // .............................................................................

    // ...................................................... Simulation ..........
    // ............................................................................
    // ............................................................................
    public class AgentBodySimulation
    {
        public AgentBody[] Agents;
        RTree agentsRTree;
        public double searchRadius;
        public double globalDeviation;
        public double deviationThreshold;

        public AgentBodySimulation(List<Plane> Pl, List<Polyline> body, double searchRadius)
        {
            Agents = new AgentBody[Pl.Count];
            agentsRTree = new RTree();
            this.searchRadius = searchRadius;
            deviationThreshold = 0.01; // to be implemented in the future (stop when global deviation under this threshold)

            // . . . . . . . . . . . . . . . . . . . . . . . . build agents array & RTree
            // since agents are fixed it can be built in advance
            for (int i = 0; i < Pl.Count; i++)
            {
                Agents[i] = new AgentBody(Pl[i], body);
                agentsRTree.Insert(Agents[i].agentPlane.Origin, i);
            }

            // . . . . . . . . . . . . . . . . . . . . . . . . . . . build neighbours map
            FindNeighbours(this.searchRadius);
            foreach (AgentBody ab in Agents)
                ab.FindTipNeighbour();
        }

        public AgentBodySimulation() { }

        public void Update()
        {
            globalDeviation = 0f;

            // foreach (AgentBody ab in Agents)
            Parallel.ForEach(Agents, ab =>
            {
                ab.Update();
            });

            // call tips UpdatePosition and UpdateDirection
            Parallel.ForEach(Agents, ab =>
            {
                foreach (Tip t in ab.body.Tips) t.Update();
            });

            // rebuild bodies
            Parallel.ForEach(Agents, ab =>
            {
                ab.RebuildBody();
            });
        }

        public void FindNeighbours(double searchRadius)
        {

            foreach (AgentBody ab in Agents)
            {
                EventHandler<RTreeEventArgs> rTreeCallback = (object sender, RTreeEventArgs args) =>
                {
                    if (Agents[args.Id] != ab)
                        ab.Neighbours.Add(Agents[args.Id]);

                };

                agentsRTree.Search(new Sphere(ab.body.O, searchRadius), rTreeCallback);
            }
        }
    }

    // ...................................................... Agent Body ..........
    // ............................................................................
    // ............................................................................

    public class AgentBody
    {
        public Plane agentPlane;
        public Body body;
        public List<AgentBody> Neighbours;
        public AgentBodySimulation ABS;

        public AgentBody(Plane agentPlane, List<Polyline> polylines)
        {
            // define agent body and orient it
            this.agentPlane = agentPlane;
            body = new Body(polylines);
            OrientBody(Plane.WorldXY, this.agentPlane);

            Neighbours = new List<AgentBody>(); // this list will be filled later by the simulation
        }

        public void OrientBody(Plane oldPlane, Plane newPlane)
        {
            var x = Transform.PlaneToPlane(oldPlane, newPlane);

            body.O.Transform(x);

            for (int i = 0; i < body.Arms.Count; i++)
            {
                body.Arms[i].Transform(x);
                body.Tips[i].pos = body.Arms[i][body.Arms[i].Count - 1];
                body.Tips[i].prev = body.Arms[i][body.Arms[i].Count - 2];
            }

            //for (int i = 0; i < body.Tips.Count; i++)
            //    body.Tips[i].pos.Transform(x);
        }

        public void RebuildBody()
        {
            for (int i = 0; i < body.Arms.Count; i++)
            {
                body.Arms[i][body.Arms[i].Count - 1] = body.Tips[i].pos;
            }

        }

        public void Update()
        {
            //TipsCohesion();
            TipsCohesionFixed();
        }

        public void FindTipNeighbour()
        {
            //Tip nearTip;
            double dSqPos, dSqPrev;
            double anglePos,anglePrev;
            double minD;
            foreach (Tip tip in body.Tips)
            {
                minD = 1.0f; //Double.MaxValue; // max Search Radius (squared)
                foreach (AgentBody nearAg in Neighbours)
                {
                    foreach (Tip neighTip in nearAg.body.Tips)
                    {
                        // calculate angles
                        anglePos = Vector3d.VectorAngle(tip.pos-tip.prev, neighTip.pos - tip.pos);
                        anglePrev = Vector3d.VectorAngle(tip.pos-tip.prev, neighTip.prev - tip.pos);
                        if (anglePos < anglePrev && anglePos < tip.angleOfVis)
                        {
                            dSqPos = tip.pos.DistanceToSquared(neighTip.pos);
                            if (dSqPos < minD)
                            {
                                minD = dSqPos;
                                tip.neighbour = neighTip;
                            }
                        } else if (anglePrev < tip.angleOfVis)
                        {
                            dSqPrev = tip.pos.DistanceToSquared(neighTip.prev);
                            if (dSqPrev < minD)
                            {
                                minD = dSqPrev;
                                tip.neighbour = neighTip;
                            }
                        }
                        // calculate distance to prev
                        //dSqPos = tip.pos.DistanceToSquared(neighTip.pos);
                        //dSqPrev = tip.pos.DistanceToSquared(neighTip.prev);
                        //if (dSqPos < minD || dSqPrev < minD)
                        //{
                        //    minD = dSqPos<dSqPrev? dSqPos:dSqPrev;
                        //    tip.neighbour = neighTip;
                        //}
                    }
                }
            }


        }

        //public void TipsCohesion()
        //{
        //    List<Tip> nearTips = new List<Tip>();

        //    // search neighbour agents and add tips to neighbours tips list (do it at every iteration since tips can move)
        //    foreach (AgentBody nearAg in Neighbours)
        //        nearTips.AddRange(nearAg.body.Tips);

        //    // call tips ComputeDesired
        //    foreach (Tip t in body.Tips) t.ComputeDesiredToPrev(nearTips);
        //}

        public void TipsCohesionFixed()
        {
            foreach (Tip t in body.Tips) t.ComputeDesiredToFixed();
        }


        public void UpdateArms()
        {

        }

        public List<Polyline> ExtractBody()
        {
            return body.Arms;
        }

    }

    // ............................................................ Body ..........
    // ............................................................................
    // ............................................................................
    public class Body
    {
        public List<Polyline> Arms;
        public List<Tip> Tips;
        public Point3d O;
        public Body(List<Polyline> polylines)
        {
            // ATTENTION - typical reference type mistake - shallow vs deep copies
            // this is a shallow copy, so it references always the same PolyLine list!
            //Arms = new List<Polyline>(polylines); 

            // this makes a deep copy - CORRECT
            Arms = new List<Polyline>();
            foreach (Polyline p in polylines)
                Arms.Add(p.Duplicate());

            O = Arms[0][0];

            Tips = new List<Tip>();
            Point3d prev;
            int armIndex = 0;
            foreach (Polyline arm in Arms)
            {
                if (arm.Count < 3) prev = O; else prev = arm[arm.Count - 2];
                Tips.Add(new Tip(this, arm[arm.Count - 1], prev, armIndex));
            }
        }
    }

    // ............................................................. Tip ..........
    // ............................................................................
    // ............................................................................
    public class Tip
    {
        public Body body;
        public Point3d pos;
        public Point3d prev;
        public Vector3d dir;
        public Vector3d desired;
        public Tip neighbour;
        public int armIndex;
        public double maxLen;
        public double cR;
        public double cI;
        public double angleOfVis;

        public Tip(Body body, Point3d pos, Point3d prev, int armIndex, double cR, double cI, double angleOfVis)
        {
            this.body = body;
            this.pos = pos;
            this.prev = prev;
            this.armIndex = armIndex;
            this.dir = pos - prev;
            this.maxLen = dir.Length * 2.0f;
            this.cR = cR;
            this.cI = cI;
            this.angleOfVis = angleOfVis;
            desired = Vector3d.Zero;
        }

        // constructor chaining example
        public Tip(Body body, Point3d pos, Point3d prev, int armIndex) : this(body, pos, prev, armIndex, 1.0f, 0.05f, Math.PI * 0.3) { }

        public void ComputeDesiredToTip(List<Tip> neighbourTips)
        {
            // reset desired vector
            desired = Vector3d.Zero;
            // if neighbour tips list is zero or null return
            if (neighbourTips == null || neighbourTips.Count == 0) return;

            Vector3d cohesion = Vector3d.Zero;
            double nTipCount = 0.0;
            double distSq, angle;
            Vector3d toNTip;
            // search neighbour tips list
            foreach (Tip nTip in neighbourTips)
            {
                distSq = pos.DistanceToSquared(nTip.pos);
                //  if tip in radius 
                if (distSq < cR)
                {
                    //   then if tip in angle of vision
                    //toTip = pos - body.O; // now we use dir
                    //toNTip = nTip.pos - body.O;
                    toNTip = nTip.pos - pos;
                    angle = Vector3d.VectorAngle(dir, toNTip);
                    if (angle < angleOfVis)
                    {
                        //     add to cohesion vector and increase counter
                        cohesion += toNTip;
                        nTipCount++;
                    }
                }
            }
            // if found tips number > 0
            if (nTipCount > 0)
            {
                // average cohesion vector
                cohesion /= nTipCount;
                // desired is cohesion
                desired = cohesion;
            }

        }

        public void ComputeDesiredToPrev(List<Tip> neighbourTips)
        {
            // reset desired vector
            desired = Vector3d.Zero;
            // if neighbour tips list is zero or null return
            if (neighbourTips == null || neighbourTips.Count == 0) return;

            Vector3d cohesion = Vector3d.Zero;
            double nTipCount = 0.0;
            double distSq, angle;
            Vector3d toNTip;
            // search neighbour tips list

            foreach (Tip nTip in neighbourTips)
            {
                distSq = pos.DistanceToSquared(nTip.prev);
                //  if prev in radius 
                if (distSq < cR)
                {
                    //   then if tip in angle of vision
                    toNTip = nTip.prev - pos;
                    angle = Vector3d.VectorAngle(dir, toNTip);
                    if (angle < angleOfVis)
                    {
                        //     add to cohesion vector and increase counter
                        cohesion += toNTip;
                        nTipCount++;
                    }
                }
            }
            // if found tips number > 0
            if (nTipCount > 0)
            {
                // average cohesion vector
                cohesion /= nTipCount;
                // desired is cohesion
                desired = cohesion;
            }

        }

        public void ComputeDesiredToFixed()
        {
            // reset desired vector
            desired = Vector3d.Zero;
            // if neighbour tips list is zero or null return
            if (neighbour == null) return;

            desired = neighbour.prev - pos;

        }

        public void UpdatePosition()
        {
            pos += desired * cI;
            // limit tip position to a max reach
            Vector3d toTip = pos - prev;
            if (toTip.Length > maxLen)
            {
                toTip = Limit(toTip, maxLen);
                pos = prev + toTip;
            }
        }

        //public void UpdatePositionOld()
        //{
        //    pos += desired * cI;
        //    // limit tip position to a max reach
        //    Vector3d toTip = pos - body.O;
        //    if (toTip.SquareLength > 1f)
        //    {
        //        toTip = Limit(toTip, 1f);
        //        pos = body.O + toTip;
        //    }
        //}

        public void UpdateDirection()
        {
            dir = pos - prev;
        }

        public void Update()
        {
            UpdatePosition();
            UpdateDirection();
        }
    }

    // .................................................................. Utilities 
    // .............................................................................

    public static Vector3d Limit(Vector3d v, double length)
    {
        Vector3d rv = new Vector3d(v);
        double len = rv.Length;
        if (len > length)
        {
            rv.Unitize();
            rv *= length;
        }
        return rv;
    }

    // </Custom additional code> 

    private List<string> __err = new List<string>(); //Do not modify this list directly.
    private List<string> __out = new List<string>(); //Do not modify this list directly.
    private RhinoDoc doc = RhinoDoc.ActiveDoc;       //Legacy field.
    private IGH_ActiveObject owner;                  //Legacy field.
    private int runCount;                            //Legacy field.

    public override void InvokeRunScript(IGH_Component owner, object rhinoDocument, int iteration, List<object> inputs, IGH_DataAccess DA)
    {
        //Prepare for a new run...
        //1. Reset lists
        this.__out.Clear();
        this.__err.Clear();

        this.Component = owner;
        this.Iteration = iteration;
        this.GrasshopperDocument = owner.OnPingDocument();
        this.RhinoDocument = rhinoDocument as Rhino.RhinoDoc;

        this.owner = this.Component;
        this.runCount = this.Iteration;
        this.doc = this.RhinoDocument;

        //2. Assign input parameters
        List<Plane> Pl = null;
        if (inputs[0] != null)
        {
            Pl = GH_DirtyCaster.CastToList<Plane>(inputs[0]);
        }
        List<int> id = null;
        if (inputs[1] != null)
        {
            id = GH_DirtyCaster.CastToList<int>(inputs[1]);
        }
        List<Polyline> body = null;
        if (inputs[2] != null)
        {
            body = GH_DirtyCaster.CastToList<Polyline>(inputs[2]);
        }


        //3. Declare output parameters
        object A = null;


        //4. Invoke RunScript
        RunScript(Pl, id, body, ref A);

        try
        {
            //5. Assign output parameters to component...
            if (A != null)
            {
                if (GH_Format.TreatAsCollection(A))
                {
                    IEnumerable __enum_A = (IEnumerable)(A);
                    DA.SetDataList(1, __enum_A);
                }
                else
                {
                    if (A is Grasshopper.Kernel.Data.IGH_DataTree)
                    {
                        //merge tree
                        DA.SetDataTree(1, (Grasshopper.Kernel.Data.IGH_DataTree)(A));
                    }
                    else
                    {
                        //assign direct
                        DA.SetData(1, A);
                    }
                }
            }
            else
            {
                DA.SetData(1, null);
            }

        }
        catch (Exception ex)
        {
            this.__err.Add(string.Format("Script exception: {0}", ex.Message));
        }
        finally
        {
            //Add errors and messages... 
            if (owner.Params.Output.Count > 0)
            {
                if (owner.Params.Output[0] is Grasshopper.Kernel.Parameters.Param_String)
                {
                    List<string> __errors_plus_messages = new List<string>();
                    if (this.__err != null) { __errors_plus_messages.AddRange(this.__err); }
                    if (this.__out != null) { __errors_plus_messages.AddRange(this.__out); }
                    if (__errors_plus_messages.Count > 0)
                        DA.SetDataList(0, __errors_plus_messages);
                }
            }
        }
    }
}