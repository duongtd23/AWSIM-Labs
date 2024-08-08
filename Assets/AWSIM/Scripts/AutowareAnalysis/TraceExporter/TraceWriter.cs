using System;
using System.Collections.Generic;
using System.IO;
using autoware_adapi_v1_msgs.msg;

namespace AWSIM.AWAnalysis.TraceExporter
{
	public class TraceWriter
	{
        private const string TAB = "  ";
        private const string TEMPLATE = "in base.maude\n\nmod TRACE is " +
            "\n  pr AWSTATE .\n\n  eq init = ";

        public TraceWriter(string filePath)
		{
			//AllMsg = new List<DynamicObjectArray>();
			contents = TEMPLATE;
			this.filePath = filePath;
			lastObjStr = "";
		}

		//public List<DynamicObjectArray> AllMsg { get; set; }
		private string contents;
		private string filePath;
		private string lastObjStr;

		public bool WriteFile()
		{
            contents += "\nendm";
            File.WriteAllText(filePath, contents);
            return true;
        }

		public bool AppendGroundTrustInfo(string stateInfo)
		{
            if (lastObjStr == "")
            {
                contents += stateInfo + " .";
            }
            else
            {
                contents += "\n" + TAB +
                    "rl " + lastObjStr + "\n" + TAB + "=> " +
                    stateInfo + " .";
            }
            lastObjStr = stateInfo;
            return true;
        }

        public bool AppendMsg(DynamicObjectArray msg)
		{
			//AllMsg.Add(msg);
			string msg2Str = WriteTimeStamp(msg.Header.Stamp);
			msg2Str += " # ";
			if (msg.Objects.Length < 1)
				return false;

			msg2Str += WriteObjects(msg.Objects);

            if (lastObjStr == "")
			{
                contents += msg2Str + " .";
            }
			else
			{
				contents += "\n" + TAB +
					"rl " + lastObjStr + "\n" + TAB + "=> " +
					msg2Str + " .";
			}
			lastObjStr = msg2Str;
			return true;
		}

		private string WriteTimeStamp(builtin_interfaces.msg.Time time)
		{
			return "time(" + time.Sec + ", " + time.Nanosec + ")";
		}

		private string WriteObjects(DynamicObject[] objects)
		{
			string result = WriteObject(objects[0]);
            for (int i = 1; i < objects.Length; i++)
			{
				result += ", " + WriteObject(objects[i]);
			}
			return "{" + result + "}";
		}

		private string WriteObject(DynamicObject obj)
		{
			string id = "id:";
			for (int i = 0; i < obj.Id.Uuid.Length; i++)
			{
				int temp = obj.Id.Uuid[i];
				id += " " + temp;
			}
			string name = "name: nil";
			string pose = "pose: " + PoseToString(obj.Kinematics.Pose);
			string twist = "twist: " + TwistToString(obj.Kinematics.Twist);
			string accel = "accel: " + AccelToString(obj.Kinematics.Accel);

            return "{" + id + ", " + name + ", " + pose + ", " + twist + ", " + accel + "}";
		}

		private string PoseToString(geometry_msgs.msg.Pose pose)
		{
            string result = "{pos: ";
            result += DoubleToMaudeString(pose.Position.X) + " ";
            result += DoubleToMaudeString(pose.Position.Y) + " ";
            result += DoubleToMaudeString(pose.Position.Z) + ", qua: ";
            result += DoubleToMaudeString(pose.Orientation.X) + " ";
            result += DoubleToMaudeString(pose.Orientation.Y) + " ";
            result += DoubleToMaudeString(pose.Orientation.Z) + " ";
            result += DoubleToMaudeString(pose.Orientation.W) + "}";
			return result;
        }

        private string TwistToString(geometry_msgs.msg.Twist twist)
        {
            string result = "{lin: ";
            result += DoubleToMaudeString(twist.Linear.X) + " ";
            result += DoubleToMaudeString(twist.Linear.Y) + " ";
            result += DoubleToMaudeString(twist.Linear.Z) + ", ang: ";
            result += DoubleToMaudeString(twist.Angular.X) + " ";
            result += DoubleToMaudeString(twist.Angular.Y) + " ";
            result += DoubleToMaudeString(twist.Angular.Z) + "}";
            return result;
        }

        private string AccelToString(geometry_msgs.msg.Accel twist)
        {
            string result = "{lin: ";
            result += DoubleToMaudeString(twist.Linear.X) + " ";
            result += DoubleToMaudeString(twist.Linear.Y) + " ";
            result += DoubleToMaudeString(twist.Linear.Z) + ", ang: ";
            result += DoubleToMaudeString(twist.Angular.X) + " ";
            result += DoubleToMaudeString(twist.Angular.Y) + " ";
            result += DoubleToMaudeString(twist.Angular.Z) + "}";
            return result;
        }

		public static string DoubleToMaudeString(double number)
		{
			string str = number.ToString();
			if (!str.Contains("."))
				str += ".0";
			return str;
		}
    }
}
