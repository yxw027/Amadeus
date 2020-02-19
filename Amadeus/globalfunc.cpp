#include "stdafx.h"
#include "globalfunc.h"
#include "EBAS_V_UI_R.h"
#include "msql.h"

EBAS_V_UI_RApp theApp;

static void clearnetlist()
{
	for (NetList::iterator i = theApp.m_NetList.begin(); i != theApp.m_NetList.end(); i++)
	{
		for (SiteList::iterator j = i.value()->m_SiteList.begin(); j != i.value()->m_SiteList.end(); j++)
		{
			if (j.value()->psock.state)
			{
				j.value()->close();
			}
			delete j.value();
		}
		i.value()->m_SiteList.clear();
	}
	theApp.m_NetList.clear();
}

void UpdateNetList()
{
	clearnetlist();
	NetList tNetList;
	QString cstr;
	//try
	{
		QStringList   cstrList;

		// 遍历子网
		m_sql.getnetlist(cstrList);
		int size = cstrList.count();
		for (int i = 0; i < size; i++)
		{
			QString NetName = cstrList.at(i);
			NetStr* pNetStr = new NetStr();
			pNetStr->m_runflag = false;
			tNetList[NetName] = pNetStr;

			// 获取子网信息结构体
			m_sql.update_netinfo2str(NetName, &pNetStr->m_Netinfo);

			// pNet->m_Server没有被使用，已在本版本代码中被去掉
			//if (theApp.m_NetList.find(NetName) != theApp.m_NetList.end())
			//{
			//	pNet->m_Server = theApp.m_NetList.find(NetName)->second->m_Server;
			//}
			//else
			//{
			//	// start server
			//}

			// 获取子网下测站信息
			QStringList MountPointList;
			m_sql.getsitelist(NetName, MountPointList);
			int MPsize = MountPointList.count();
			for (int site_i = 0; site_i < MPsize; site_i++)
			{
				cstr = MountPointList.at(site_i);
				QStringList cstrinfo;
				m_sql.getsiteinfo(cstr, cstrinfo);
				if (cstrinfo.count())
					// 老版程序此处为cstr = cstrinfo.at(0)，会导致下一个if条件判断得不到正确结果
					cstr = cstrinfo.at(1);
				else
					continue;

				if (theApp.m_NetList.find(NetName) != theApp.m_NetList.end()
					&& theApp.m_NetList.find(NetName).value()->m_SiteList.find(cstr)
					!= theApp.m_NetList.find(NetName).value()->m_SiteList.end())
				{
					// theApp中已存在，则维持theApp中的值，不更新theApp
					tNetList[NetName]->m_SiteList[cstr] = theApp.m_NetList.find(NetName).value()->m_SiteList.find(cstr).value();
				}
				// 
				else
				{
					// theApp中不存在，则使用数据库中的记录更新theApp
					CNtripClient tNtripClient;
					//"MOUNTPOINT","SITATABLE","SITETYPE","COM_TYPE","IP","PORT","USERNAME","PASSWORD","LATITUDE","LONGITUDE","HEIGHT"
					int index = 1;
					// 获取工作路径,老程序为sprintf(temp->rtcm_fwrt.workpath, pNet->m_Netinfo.WORKPATH, pNet->m_Netinfo.WORKPATH.GetLength());
					sprintf(tNtripClient.rtcm_fwrt.workpath, pNetStr->m_Netinfo.WORKPATH.toUtf8().data());
					// 获取 所在子网名称
					tNtripClient.rtcm_fwrt.m_NetName = NetName;
					// 获取 挂载点名称
					tNtripClient.MountPoint = cstrinfo.at(index++);
					// 获取 显示标识
					tNtripClient.sitelable = cstrinfo.at(index++);
					// 获取 测站类型 1 基准站 2 移动站 3 广播星历
					cstr = cstrinfo.at(index++);
					tNtripClient.sitetype = cstr.toInt();
					// 获取 连接类型  0=TCP CLIENT  1=TCP SERVER  2=NTRIP CLIENT
					cstr = cstrinfo.at(index++);
					tNtripClient.m_ConnectType = (ConnectType)(cstr.toInt());
					// 获取 IP地址
					tNtripClient.IP = cstrinfo.at(index++);
					// 获取 PORT端口号
					tNtripClient.Port = cstrinfo.at(index++);
					// 获取 用户名
					tNtripClient.User = cstrinfo.at(index++);
					// 获取 密码
					tNtripClient.Pwd = cstrinfo.at(index++);
					// 获取 测站经纬度
					cstr = cstrinfo.at(index++);
					tNtripClient.XYZ[0] = cstr.toDouble();;
					cstr = cstrinfo.at(index++);
					tNtripClient.XYZ[1] = cstr.toDouble();
					cstr = cstrinfo.at(index++);
					tNtripClient.XYZ[2] = cstr.toDouble();
					ecef2pos(tNtripClient.XYZ, tNtripClient.BLH);
					tNtripClient.netname = NetName;

					// 将tNtripClient插入到tNetList的SiteList中
					tNetList[tNtripClient.rtcm_fwrt.m_NetName]->m_SiteList.insert(tNtripClient.MountPoint.toUpper(), &tNtripClient);
				}		
			}
		}
	}
	//catch (CDBException* e)
	//{
	//	cstr.Format("ERROR: void CMainFrame::UpdateNetList(void)   %s ", e->m_strError);
	//}

	// 遍历原列表，删除列表多余的测站
	NetList::iterator Net_i(theApp.m_NetList.begin());
	while (Net_i != theApp.m_NetList.end())
	{
		QMap<QString, CNtripClient*>::iterator site_i(Net_i.value()->m_SiteList.begin());
		while (site_i != Net_i.value()->m_SiteList.end())
		{
			// 如果找着则把临时中的删除，否则删除原列表中多余的(因为迭代器的原因不能直接删除)
			if (!(tNetList.find(Net_i.key()) != tNetList.end() && tNetList.find(Net_i.key()).value()->m_SiteList.find(site_i.key()) != tNetList.find(Net_i.key()).value()->m_SiteList.end()))
			{
				CNtripClient* pClient = (CNtripClient*)site_i.value();
				pClient->~CNtripClient();
				site_i.value() = NULL;
			}
			site_i++;
		}
		Net_i++;
	}
	// 删除指针被置空的列表
	theApp.m_NetList.clear();
	for(Net_i= tNetList.begin(); Net_i != tNetList.end(); Net_i++)
	{ 
		theApp.m_NetList.insert(Net_i.key(), Net_i.value());
	}
}

